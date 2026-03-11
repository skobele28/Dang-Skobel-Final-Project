#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include <sys/time.h>
#include "esp_idf_lib_helpers.h"
#include "hd44780.h"
#include <inttypes.h>
#include <stdio.h>
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/ledc.h"


//Global variables -- Can change number of floors and the LDR value for middle light level
#define floors              3       // three-story elevator
#define LDR_mid            1200     // LDR adc-bit value to separate light and dark (calibrated)
// arrays for elevator pushbutton requests and LDR values, initialize all to 0
int inside_req[floors+1] = {0};
int up_call [floors+1] = {0};
int down_call [floors+1] = {0};
int LDR_values[floors+1] = {0};

int current_floor = 1;              // variable to track current floor, initialize to floor 1


//Pins declaration -- change the GPIO for each input/ output accordingly
#define FLOOR1_SELECT       GPIO_NUM_6           
#define FLOOR2_SELECT       GPIO_NUM_5
#define FLOOR3_SELECT       GPIO_NUM_4
#define FLOOR1_CALLUP       GPIO_NUM_17
#define FLOOR2_CALLDOWN     GPIO_NUM_16
#define FLOOR2_CALLUP       GPIO_NUM_15
#define FLOOR3_CALLDOWN     GPIO_NUM_7
#define TEMP_SENSOR         GPIO_NUM_8
#define FIRE_SYSTEM         GPIO_NUM_9

//LCD structure, including GPIOs
hd44780_t lcd = {
    .write_cb = NULL,
    .font = HD44780_FONT_5X8,
    .lines = 2,
    .pins = {
        .rs = GPIO_NUM_1,                           // GPIO for Register Select
        .e  = GPIO_NUM_48,                          // GPIO for enable
        .d4 = GPIO_NUM_35,                          // GPIO for data 4
        .d5 = GPIO_NUM_36,                          // GPIO for data 5
        .d6 = GPIO_NUM_37,                          // GPIO for data 6
        .d7 = GPIO_NUM_2,                           // GPIO for data 7
        .bl = HD44780_NOT_USED
    }
};


// character data for up arrow and down arrow custom characters
char LCD_string[8];
static const uint8_t char_data[] =
{
    0x04, 0x0E, 0x15, 0x04, 0x04, 0x04, 0x04, 0x04,
    0x04, 0x04, 0x04, 0x04, 0x04, 0x15, 0x0E, 0x04
};


//LEDC configuration
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO      (18)        //GPIO for the LEDC .
#define LEDC_CHANNEL        LEDC_CHANNEL_0
#define LEDC_DUTY_RES       LEDC_TIMER_13_BIT 
#define LEDC_FREQUENCY      (50)  // Frequency in Hertz.
#define go_up_max           (670) // maximum speed for elevator moving up
#define go_down_max         (540) // maximum speed for elevator moving down
#define stop                (614) // duty value for stopped elevator

//ADC configuration
int FLOOR_LDR[floors+1] = {-1, 0, 1, 2};
#define ADC_ATTEN       ADC_ATTEN_DB_12 // set ADC attenuation
#define BITWIDTH        ADC_BITWIDTH_12 // set ADC bitwidth
adc_oneshot_unit_handle_t adc2_handle;  // ADC handle for Mode and Timer
#define ACTIVE          1


//function and task prototypes

// function to configure buttons, inputs, and outputs
static void button_config(void);
// function to configure ADC
static void ADC_config(void);
// function to configure and initialize LEDC
static void ledc_initialize(void);
// task gets level from each push button and set values in arrays accordingly
static void input_task(void *pvParameter);
// FSM for elevator logic
static void elevator_FSM(void *pvParameter);
// task to control servo motor (stop, move up, move down, slow down)
static void servo_task(void *pvParameter);
// function to determine there are no requests
static bool all_zeroes(void);
// function to determine if there are any requests for floor(s) above the current floor
static bool req_up(void);
// function to detemrine if there are any requests for floor(s) below the current floor
static bool req_down(void);
// function to determine if there is a request for the current floor
static bool floor_req(int f);


void app_main(void)
{
    // call functions to configure hardware (buttons, ADC, LEDC, LCD)
    button_config();
    ADC_config();
    ledc_initialize();
    ESP_ERROR_CHECK(hd44780_init(&lcd));
    hd44780_upload_character(&lcd, 0, char_data);
    hd44780_upload_character(&lcd, 1, char_data + 8);

    // create tasks
    xTaskCreate(input_task, "Input Task", 2048, NULL, 3, NULL);
    xTaskCreate(servo_task, "Servo Task", 2048, NULL, 4, NULL);
    xTaskCreate(elevator_FSM, "Elevator FSM", 2048, NULL, 5, NULL);
    

    while (1){
        
        // adc oneshot read for three LDRs, set current floor variable based on LDR readings
        for (int i=1; i<= floors; i++) {
            int adc_bits;
            adc_oneshot_read (adc2_handle, FLOOR_LDR[i], &adc_bits);
            LDR_values[i] = adc_bits;
            if (adc_bits < LDR_mid && i != current_floor){
                current_floor = i;
            }
        }

        // print current floor on LCD screen
        hd44780_gotoxy(&lcd, 0, 0);
        snprintf(LCD_string, sizeof(LCD_string), "Floor %d", current_floor);
        hd44780_puts(&lcd, LCD_string);
        vTaskDelay(pdMS_TO_TICKS(100));
        /*printf("%d, %d, %d\n", LDR_values[1], LDR_values[2], LDR_values[3]);
        int temp_sensor = gpio_get_level(TEMP_SENSOR);
        printf("%d\n", temp_sensor);*/
    }
}

// task gets level from each push button and sets corresponding array entry to 1
void input_task (void *pvParameter) {
    while(1){
        if (gpio_get_level(FLOOR1_SELECT)==ACTIVE) {inside_req[1] = 1;}
        if (gpio_get_level(FLOOR2_SELECT)==ACTIVE) {inside_req[2] = 1;}
        if (gpio_get_level(FLOOR3_SELECT)==ACTIVE) {inside_req[3] = 1;}
        if (gpio_get_level(FLOOR1_CALLUP)==ACTIVE) {up_call[1] = 1;}
        if (gpio_get_level(FLOOR2_CALLUP)==ACTIVE) {up_call[2] = 1;}
        if (gpio_get_level(FLOOR2_CALLDOWN)==ACTIVE) {down_call[2] = 1;}
        if (gpio_get_level(FLOOR3_CALLDOWN)==ACTIVE) {down_call[3] = 1;}
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

// initialize FSM states as global variables
typedef enum {
        Idle,
        Moveup,
        Movedown,
        Slow,
        Wait,
        Fire,
    } State_t;
    State_t state;
    State_t last_state;

// variables to control fire system
int fire = 0;
int blink = 1;

void elevator_FSM (void *pvParameter) {
    state = Idle;                                   // initial state is Idle
    while (1) {
        last_state = state;                         // store last_state
        vTaskDelay (20/portTICK_PERIOD_MS);
        switch (state) {                            // switch statement based on value of state
            case (Idle):
                if (gpio_get_level(TEMP_SENSOR)){   // if temp sensor input is HIGH, switch to Fire state
                    state = Fire;
                    break;
                }
                else if (all_zeroes()){             // if no button presses, remain in Idle state
                    state = Idle;
                }
                else if (req_up()) {                // if there is request to go to floor above current floor, switch to Moveup state
                    state = Moveup;
                }
                else if (req_down()) {              // if there is request to to to floor below current floor, switch to Movedown state
                    state = Movedown;
                }
                else {                              // Otherwise (ex. request for current floor), switch to Wait state
                    state = Wait;
                }
                hd44780_gotoxy(&lcd, 14, 0);        // print nothing in arrow location on LCD
                hd44780_puts(&lcd, " ");
                break;
            
            case (Wait):
                if (gpio_get_level(TEMP_SENSOR)){   // if temp sensor input is HIGH, switch to Fire state
                    state = Fire;
                    break;
                }
                vTaskDelay (2000/portTICK_PERIOD_MS);   // delay 2s (wait for people to enter/exit elevator)
                inside_req[current_floor] = 0;          // clear inside request
                
                // prioritize moving in same direction as last_state
                if (last_state == Moveup) {             // if last_state is Moveup
                    if (req_up()) state = Moveup;       // if there is another up request, switch to Moveup state
                    else if (req_down()) state = Movedown;  // if there is a down request, switch to Movedown state
                    else state = Idle;                  // if there are no requests to move to other floors, switch to Idle state
                    up_call[current_floor] = 0;         // clear up_call for current floor
                }
                else if (last_state == Movedown) {      //if last_state is Movedown
                    if (req_down()) state = Movedown;   // if there is another down request, switch to Movedown state
                    else if (req_up()) state = Moveup;  // if there is an up request, switch to Moveup state
                    else state = Idle;                  // if there are no requests to move to other floors, switch to Idle state
                    down_call[current_floor] = 0;       // clear down_call for current floor
                }
                else {
                    state = Idle;                       // switch to Idle state
                    up_call[current_floor] = 0;         // clear current floor up_call request
                    down_call[current_floor]=0;         // clear current floor down_call request
                }
                hd44780_gotoxy(&lcd, 14, 0);            // print nothing in arrow location on LCD
                hd44780_puts(&lcd, " ");
                break;
            
            case (Moveup):
                if (gpio_get_level(TEMP_SENSOR)){   // if temp sensor input is HIGH, switch to Fire state
                    state = Fire;
                    break;
                }
                if (floor_req(current_floor)) {     // if there is a floor request for the current floor, switch to Slow state
                    state = Slow;
                }
                else {                              // otherwise, remain in Moveup state
                    state = Moveup;
                }
                hd44780_gotoxy(&lcd, 14, 0);        // print up arrow on LCD
                hd44780_puts(&lcd, "\x08");
                break;

            case (Movedown):
                if (gpio_get_level(TEMP_SENSOR)){   // if temp sensor input is HIGH, switch to Fire state
                    state = Fire;
                    break;
                }
                if (floor_req(current_floor)) {     // if there is a floor request for the current floor, switch to Slow state
                    state = Slow;
                }
                else {
                    state = Movedown;               // otherwise, remain in Movedown state
                }
                hd44780_gotoxy(&lcd, 14, 0);        // print down arrow on LCD
                hd44780_puts(&lcd, "\x09");
                break;
            
            case (Slow):
                if (gpio_get_level(TEMP_SENSOR)){   // if temp sensor input is HIGH, switch to Fire state
                    state = Fire;
                    break;
                }
                if(LDR_values[current_floor] > LDR_mid){    //Change state to Wait when LDR detects bright light again
                    state = Wait;
                }
                else{
                    state = Slow;                           // Otherwise, stay in Slow state
                }
                break;
            

            case (Fire):
                gpio_set_level(FIRE_SYSTEM, blink);         // blink LED and buzzer alarm
                blink = !blink;
                if (fire == 0){                             // send elevator to first floor
                    inside_req[1] = 1;
                    fire = 1;                               // change fire variable so elevator is only sent to first floor once
                }
                vTaskDelay(1000/portTICK_PERIOD_MS);        // delay 1s for blink (2s-period blinking)
                state = Fire;                               // do not leave Fire state (system must be reset)
        }
    }
}

// task to control servo motor (stopped, moving up, moving down, slowing down, fire state)
void servo_task (void *pvParameter) {
    int executed = 0;       // ensures each motor function only occurs once within the while loop, until conditions change
    if(current_floor != 1) {
        inside_req[1] = 1;
    }
    while (1){
        vTaskDelay(20/portTICK_PERIOD_MS);

        // If state is Idle or Wait, set elevator duty to stop (elevator does not move)
        if ((state == Idle || state == Wait) && executed != 1) {
            ledc_set_duty (LEDC_MODE, LEDC_CHANNEL, stop);
            ledc_update_duty (LEDC_MODE, LEDC_CHANNEL);
            executed = 1;               // change variable executed value so if-statement only sets value once
        }
        // If state is Moveup, accelerate elevator to go_up_max value
        else if (state == Moveup && executed != 2) {
            for(int i = stop; i <= go_up_max; i = i + 5){   // increment duty count by 5
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);  // set duty cycle to new i-value
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);  // update duty cycle
                vTaskDelay(10/portTICK_PERIOD_MS);          // wait 10 ms
            }
            executed = 2;               // change variable executed value so for-loop only runs once
        }

        // If state is Movedown, accelerate elevator to go_down_max value
        else if (state == Movedown && executed != 3) {
            for(int i = stop; i >= go_down_max; i = i - 5){ // increment duty count by 5
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);  // set duty cycle to new i-value
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);  // update duty cycle
                vTaskDelay(10/portTICK_PERIOD_MS);          // wait 10 ms
            }
            executed = 3;               // change variable executed value so for-loop only runs once
        }

        // If state is Slow, decelerate elevator to 10 away from stop value
        else if (state == Slow && executed != 4) {
            if (executed == 2) {
                for(int i = ledc_get_duty(LEDC_MODE, LEDC_CHANNEL); i >= stop + 10; i--){   // increment duty count by 1
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);  // set duty cycle to new i-value
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);  // update duty cycle
                    vTaskDelay(20/portTICK_PERIOD_MS);          // wait 10 ms
                }
            }
            if (executed == 3) {
                for(int i = ledc_get_duty(LEDC_MODE, LEDC_CHANNEL); i <= stop - 10; i++){   // increment duty count by 1
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);  // set duty cycle to new i-value
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);  // update duty cycle
                    vTaskDelay(20/portTICK_PERIOD_MS);          // wait 10 ms
                }
            }
            executed = 4;               // change variable executed value so for-loop only runs once
        }

        // If state is Fire, send elevator to first floor (accelerate, then decelerate, then stop at first floor)
        else if (state == Fire) {
            if (current_floor == 1 && LDR_values[1] > LDR_mid) {vTaskDelay (pdMS_TO_TICKS(100000));}
            // if elevator is not on first floor, accelerate to go_down_max
            else if (current_floor != 1) {
                if(executed !=5){
                    for(int i = stop; i >= go_down_max; i = i - 5){ // increment duty count by 5
                        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);  // set duty cycle to new i-value
                        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);  // update duty cycle
                        vTaskDelay(10/portTICK_PERIOD_MS);          // wait 10 ms
                    }
                    executed = 5;       // change variable executed value so for-loop only runs once
                }
            }
            // once elevator reaches first floor, slow down until first floor LDR is bright again
            else {
                while (LDR_values[1] < LDR_mid) {
                    if (executed != 6) {
                        for(int i = ledc_get_duty(LEDC_MODE, LEDC_CHANNEL); i <= stop - 10; i++){   // increment duty count by 1
                            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);  // set duty cycle to new i-value
                            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);  // update duty cycle
                            vTaskDelay(20/portTICK_PERIOD_MS);          // wait 10 ms
                        }
                        executed = 6;   // change variable executed value so for-loop only runs once
                    }
                    vTaskDelay (pdMS_TO_TICKS(10));
                }
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, stop);       // stop elevator when program exits while loop (when LDR_values[1] >= LDR_mid)
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);  
            }
        }
    }
}


void button_config(void){
    //Reset pins
    gpio_reset_pin(FLOOR1_SELECT);
    gpio_reset_pin(FLOOR2_SELECT);
    gpio_reset_pin(FLOOR3_SELECT);
    gpio_reset_pin(FLOOR1_CALLUP);
    gpio_reset_pin(FLOOR2_CALLDOWN);
    gpio_reset_pin(FLOOR2_CALLUP);
    gpio_reset_pin(FLOOR3_CALLDOWN);
    gpio_reset_pin(TEMP_SENSOR);
    gpio_reset_pin(FIRE_SYSTEM);

    //Set directions
    gpio_set_direction(FLOOR1_SELECT, GPIO_MODE_INPUT);
    gpio_set_direction(FLOOR2_SELECT, GPIO_MODE_INPUT);
    gpio_set_direction(FLOOR3_SELECT, GPIO_MODE_INPUT);
    gpio_set_direction(FLOOR1_CALLUP, GPIO_MODE_INPUT);
    gpio_set_direction(FLOOR2_CALLDOWN, GPIO_MODE_INPUT);
    gpio_set_direction(FLOOR2_CALLUP, GPIO_MODE_INPUT);
    gpio_set_direction(FLOOR3_CALLDOWN, GPIO_MODE_INPUT);
    gpio_set_direction(TEMP_SENSOR, GPIO_MODE_INPUT);
    gpio_set_direction(FIRE_SYSTEM, GPIO_MODE_OUTPUT);

    //Configure pulldown
    gpio_pulldown_en(FLOOR1_SELECT);
    gpio_pulldown_en(FLOOR2_SELECT);
    gpio_pulldown_en(FLOOR3_SELECT);
    gpio_pulldown_en(FLOOR1_CALLUP);
    gpio_pulldown_en(FLOOR2_CALLDOWN);
    gpio_pulldown_en(FLOOR2_CALLUP);
    gpio_pulldown_en(FLOOR3_CALLDOWN);
    gpio_pulldown_en(TEMP_SENSOR);
}

// function to configure and initialize ADC
void ADC_config(void){
    adc_oneshot_unit_init_cfg_t init_config2 = {        // Unit configuration
        .unit_id = ADC_UNIT_2,
    };                                                  
    adc_oneshot_new_unit(&init_config2, &adc2_handle);  // Populate unit handle

    adc_oneshot_chan_cfg_t config = {                   // Channel config
        .atten = ADC_ATTEN,
        .bitwidth = BITWIDTH
    };                                                  
    
    for (int i=1; i<= floors; i++) {                    // configure the ADC channel for each LDR
        adc_oneshot_config_channel (adc2_handle, FLOOR_LDR[i], &config);
    }
}

// function to configure and initialize ledc
void ledc_initialize(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 50 Hz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = stop, // Set duty to stop initially
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
}

// if there are any inside requests or elevator calls, return false, otherwise return true
bool all_zeroes(void) {
    for (int i = 1; i <= floors; i++) {
        if (inside_req[i] != 0 || up_call[i] != 0 || down_call[i] != 0) {
            return false;
        }
    }
    return true;
}

// if there are any buttons pushed on floors above current floor, return true, otherwise return false
bool req_up(void) {
    for (int i = current_floor +1; i <= floors; i++) {
        if (inside_req[i] != 0|| up_call[i] != 0 || down_call[i] != 0) {
            return true;
        }
    }
    return false;
}

// if there are any buttons pushed on floors below current floor, return true, otherwise return false
bool req_down(void) {
    for (int i = current_floor - 1; i > 0; i--) {
        if (inside_req[i] != 0|| up_call[i] != 0 || down_call[i] != 0) {
            return true;
        }
    }
    return false;
}

// if there is a request for the current floor, return true, otherwise return false
bool floor_req(int f) {
    if (up_call[f] == 1 || down_call[f] ==1 || inside_req[f] == 1) {
        return true;
    }
    return false;
}