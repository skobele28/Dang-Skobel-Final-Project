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
#define floors              3
#define LDR_mid            1000
int inside_req[floors+1] = {0};
int up_call [floors+1] = {0};
int down_call [floors+1] = {0};
int LDR_values[floors+1] = {0};
int current_floor = 1;


//Pins declaration  -- change the GPIO for each input/ output accordingly
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
        .e  = GPIO_NUM_48,                           // GPIO for enable
        .d4 = GPIO_NUM_35,                           // GPIO for data 4
        .d5 = GPIO_NUM_36,                          // GPIO for data 5
        .d6 = GPIO_NUM_37,                          // GPIO for data 6
        .d7 = GPIO_NUM_2,                          // GPIO for data 7
        .bl = HD44780_NOT_USED
    }
};

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
#define LEDC_FREQUENCY      (50) // Frequency in Hertz.
#define go_up_max           (670) //(719)
#define go_down_max         (540) //(509)
#define stop                (614)

//ADC configuration
int FLOOR_LDR[floors+1] = {-1, 0, 1, 2};
#define ADC_ATTEN       ADC_ATTEN_DB_12 // set ADC attenuation
#define BITWIDTH        ADC_BITWIDTH_12 // set ADC bitwidth
adc_oneshot_unit_handle_t adc2_handle;      // ADC handle for Mode and Timer
#define ACTIVE          1


//function prototypes
static void button_config(void);  
static void ADC_config(void);
static void ledc_initialize(void);
static void input_task(void *pvParameter);
static void elevator_FSM(void *pvParameter);
static void servo_task(void *pvParameter);
static bool all_zeroes(void);
static bool req_up(void);
static bool req_down(void);
static bool floor_req(int f);


void app_main(void)
{
    button_config();
    ADC_config();
    ledc_initialize();
    ESP_ERROR_CHECK(hd44780_init(&lcd));
    hd44780_upload_character(&lcd, 0, char_data);
    hd44780_upload_character(&lcd, 1, char_data + 8);
    xTaskCreate(input_task, "Input Task", 2048, NULL, 3, NULL);
    xTaskCreate(servo_task, "Servo Task", 2048, NULL, 4, NULL);
    xTaskCreate(elevator_FSM, "Elevator FSM", 2048, NULL, 5, NULL);
    

    while (1){
        // adc oneshot read for three LDRs
        for (int i=1; i<= floors; i++) {
            int adc_bits;
            adc_oneshot_read (adc2_handle, FLOOR_LDR[i], &adc_bits);
            LDR_values[i] = adc_bits;
            if (adc_bits < LDR_mid && i != current_floor){
                current_floor = i;
            }
        }
        hd44780_gotoxy(&lcd, 0, 0);
        snprintf(LCD_string, sizeof(LCD_string), "Floor %d", current_floor);
        hd44780_puts(&lcd, LCD_string);
        vTaskDelay(pdMS_TO_TICKS(100));
        printf("%d, %d, %d\n", LDR_values[1], LDR_values[2], LDR_values[3]);
        int temp_sensor = gpio_get_level(TEMP_SENSOR);
        printf("%d\n", temp_sensor);
    }
}


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
int fire = 0;
int blink = 1;

void elevator_FSM (void *pvParameter) {
    state = Idle;
    while (1) {
        last_state = state;
        vTaskDelay (20/portTICK_PERIOD_MS);
        switch (state) {
            case (Idle):
            printf ("Idle\n");
                if (gpio_get_level(TEMP_SENSOR)){
                    state = Fire;
                    break;
                }
                else if (all_zeroes()){
                    state = Idle;
                }
                else if (req_up()) {
                    state = Moveup;
                }
                else if (req_down()) {
                    state = Movedown;
                }
                else {
                    state = Wait;
                }
                hd44780_gotoxy(&lcd, 14, 0);
                hd44780_puts(&lcd, " ");
                break;
            
            case (Wait):
            printf ("Wait\n");
                if (gpio_get_level(TEMP_SENSOR)){
                    state = Fire;
                    break;
                }
                vTaskDelay (2000/portTICK_PERIOD_MS);
                inside_req[current_floor] = 0;
                if (last_state == Moveup) {
                    if (req_up()) state = Moveup;
                    else if (req_down()) state = Movedown;
                    else state = Idle;
                    up_call[current_floor] = 0;
                }
                else if (last_state == Movedown) {
                    if (req_down()) state = Movedown;
                    else if (req_up()) state = Moveup;
                    else state = Idle;
                    down_call[current_floor] = 0;
                }
                else {
                    state = Idle;
                    up_call[current_floor] = 0;
                    down_call[current_floor]=0;
                }
                hd44780_gotoxy(&lcd, 14, 0);
                hd44780_puts(&lcd, " ");
                break;
            
            case (Moveup):
                if (gpio_get_level(TEMP_SENSOR)){
                    state = Fire;
                    break;
                }
            printf ("Moveup\n");
                if (floor_req(current_floor)) {
                    state = Slow;
                }
                else {
                    state = Moveup;
                }
                hd44780_gotoxy(&lcd, 14, 0);
                hd44780_puts(&lcd, "\x08");
                break;

            case (Movedown):
                if (gpio_get_level(TEMP_SENSOR)){
                    state = Fire;
                    break;
                }
            printf ("Movedown\n");
                if (floor_req(current_floor)) {
                    state = Slow;
                }
                else {
                    state = Movedown;
                }
                hd44780_gotoxy(&lcd, 14, 0);
                hd44780_puts(&lcd, "\x09");
                break;
            
            case (Slow):
                if (gpio_get_level(TEMP_SENSOR)){
                    state = Fire;
                    break;
                }
            printf ("Slow\n");
                if(LDR_values[current_floor] > LDR_mid){
                    state = Wait;
                }
                else{
                    state = Slow;
                }
                break;
            //Change state to stop when the LDR detect bright light again.

            case (Fire):
                printf("Fire state\n");
                gpio_set_level(FIRE_SYSTEM, blink);
                blink = !blink;
                if (fire == 0){
                    inside_req[1] = 1;
                    fire = 1;
                }
                vTaskDelay(1000/portTICK_PERIOD_MS);
                state = Fire;
        }
    }
}


void servo_task (void *pvParameter) {
    int executed = 0;       // ensures each motor function only occurs once within the while loop, until conditions change
    if(current_floor != 1) {
        inside_req[1] = 1;
    }
    while (1){
        vTaskDelay(20/portTICK_PERIOD_MS);
        if ((state == Idle || state == Wait) && executed != 1) {
            ledc_set_duty (LEDC_MODE, LEDC_CHANNEL, stop);
            ledc_update_duty (LEDC_MODE, LEDC_CHANNEL);
            executed = 1;
        }
        else if (state == Moveup && executed != 2) {
            for(int i = stop; i <= go_up_max; i = i + 5){   // increment duty count by 5
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);              // set duty cycle to new i-value
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);              // update duty cycle
                vTaskDelay(10/portTICK_PERIOD_MS);         // wait 10 ms
            }
            executed = 2;
        }
        else if (state == Movedown && executed != 3) {
            for(int i = stop; i >= go_down_max; i = i - 5){   // increment duty count by 5
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);              // set duty cycle to new i-value
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);              // update duty cycle
                vTaskDelay(10/portTICK_PERIOD_MS);         // wait 10 ms
            }
            executed = 3;
        }
        else if (state == Slow && executed != 4) {
            if (executed == 2) {
                for(int i = ledc_get_duty(LEDC_MODE, LEDC_CHANNEL); i >= stop + 10; i--){
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);              // set duty cycle to new i-value
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);              // update duty cycle
                    vTaskDelay(20/portTICK_PERIOD_MS);         // wait 10 ms
                }
            }
            if (executed == 3) {
                for(int i = ledc_get_duty(LEDC_MODE, LEDC_CHANNEL); i <= stop - 10; i++){
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);              // set duty cycle to new i-value
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);              // update duty cycle
                    vTaskDelay(20/portTICK_PERIOD_MS);         // wait 10 ms
                }
            }
            executed = 4;
        }   
        else if (state == Fire) {
            if (current_floor == 1 && LDR_values[1] > LDR_mid) {vTaskDelay (pdMS_TO_TICKS(100000));}
            else if (current_floor != 1) {
                if(executed !=5){
                    for(int i = stop; i >= go_down_max; i = i - 5){   // increment duty count by 5
                        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);              // set duty cycle to new i-value
                        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);              // update duty cycle
                        vTaskDelay(10/portTICK_PERIOD_MS);         // wait 10 ms
                    }
                    executed = 5;
                }
            }
            else {
                while (LDR_values[1] < LDR_mid) {
                    if (executed != 6) {
                        for(int i = ledc_get_duty(LEDC_MODE, LEDC_CHANNEL); i <= stop - 10; i++){
                            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);              // set duty cycle to new i-value
                            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);              // update duty cycle
                            vTaskDelay(20/portTICK_PERIOD_MS);         // wait 10 ms
                        }
                        executed = 6;
                    }
                    vTaskDelay (pdMS_TO_TICKS(10));
                }
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, stop);           
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);  
            }


            /*if (executed != 5){
                for(int i = stop; i >= go_down_max; i = i - 5){   // increment duty count by 5
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);              // set duty cycle to new i-value
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);              // update duty cycle
                    vTaskDelay(10/portTICK_PERIOD_MS);         // wait 10 ms
                }
                executed = 5;
            }

            if (LDR_values[1] < LDR_mid){
                for(int i = ledc_get_duty(LEDC_MODE, LEDC_CHANNEL); i <= stop - 10; i++){
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);              // set duty cycle to new i-value
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);              // update duty cycle
                    vTaskDelay(20/portTICK_PERIOD_MS);         // wait 10 ms
                    if (LDR_values[1] > LDR_mid){
                        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, stop);
                        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                        executed = 6;
                        break;
                    }

                }
            }*/
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
}

// function to configure and initialize ADC
void ADC_config(void){
    adc_oneshot_unit_init_cfg_t init_config2 = {
        .unit_id = ADC_UNIT_2,
    };                                                  // Unit configuration
    adc_oneshot_new_unit(&init_config2, &adc2_handle);  // Populate unit handle

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN,
        .bitwidth = BITWIDTH
    };                                                  // Channel config
    
    for (int i=1; i<= floors; i++) {
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


bool all_zeroes(void) {
    for (int i = 1; i <= floors; i++) {
        if (inside_req[i] != 0 || up_call[i] != 0 || down_call[i] != 0) {
            return false;
        }
    }
    return true;
}

bool req_up(void) {
    for (int i = current_floor +1; i <= floors; i++) {
        if (inside_req[i] != 0|| up_call[i] != 0 || down_call[i] != 0) {
            return true;
        }
    }
    return false;
}

bool req_down(void) {
    for (int i = current_floor - 1; i > 0; i--) {
        if (inside_req[i] != 0|| up_call[i] != 0 || down_call[i] != 0) {
            return true;
        }
    }
    return false;
}

bool floor_req(int f) {
    if (up_call[f] == 1 || down_call[f] ==1 || inside_req[f] == 1) {
        return true;
    }
    return false;
}