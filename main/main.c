#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include <sys/time.h>
#include "../components/LCD/esp-idf-lib__esp_idf_lib_helpers/esp_idf_lib_helpers.h"
#include "../components/LCD/esp-idf-lib__hd44780/hd44780.h"
#include "../components/temp_sensor/DHT.h"
#include <inttypes.h>
#include <stdio.h>
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/ledc.h"

//Global variables
//Pins declaration
#define FLOOR1_SELECT   GPIO_NUM_6           
#define FLOOR2_SELECT   GPIO_NUM_5
#define FLOOR3_SELECT   GPIO_NUM_4
#define FLOOR1_CALLUP   GPIO_NUM_17
#define FLOOR2_CALLDOWN   GPIO_NUM_16
#define FLOOR2_CALLUP GPIO_NUM_15
#define FLOOR3_CALLDOWN GPIO_NUM_7
#define TEMP_SENSOR     GPIO_NUM_8
#define FIRE_SYSTEM     GPIO_NUM_9

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
        .d7 = GPIO_NUM_38,                          // GPIO for data 7
        .bl = HD44780_NOT_USED
    }
};

//LEDC configuration
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO      (18)        //GPIO for the LEDC .
#define LEDC_CHANNEL        LEDC_CHANNEL_0
#define LEDC_DUTY_RES       LEDC_TIMER_13_BIT 
#define LEDC_FREQUENCY      (50) // Frequency in Hertz.
#define go_up_max               ()
#define go_down_max             ()

//ADC configuration
#define FLOOR1_LDR      ADC_CHANNEL_2   // LDR sensor (auto headlight) ADC1 channel 0
#define FLOOR2_LDR      ADC_CHANNEL_1   // LDR sensor (auto headlight) ADC1 channel 0
#define FLOOR3_LDR      ADC_CHANNEL_0   // LDR sensor (auto headlight) ADC1 channel 0
#define ADC_ATTEN       ADC_ATTEN_DB_12 // set ADC attenuation
#define BITWIDTH        ADC_BITWIDTH_12 // set ADC bitwidth
adc_oneshot_unit_handle_t adc2_handle;      // ADC handle for Mode and Timer

//Global boolean values
bool floor1_present = true;
bool floor2_present = false;
bool floor3_present = false;
bool floor1_callup = false;
bool floor2_calldown = false;
bool floor2_callup = false;
bool floor3_calldown = false;
bool floor1_select = false;
bool floor2_select = false;
bool floor3_select = false;

//function prototypes
static void button_config(void);  
static void ADC_config(void);
static void ledc_initialize(void);
static void input_task(void);
static void servo_task(void);
static void floor_logic(void);

void app_main(void)
{

    button_config();
    ADC_config();
    ledc_initialize();
    ESP_ERROR_CHECK(hd44780_init(&lcd));

    // adc oneshot read and calibration configuration
    int floor1_adc_bits;                       // potentiometer ADC reading (bits)
    int floor1_adc_mV;                         // potentiometer ADC reading (mV)
    int floor2_adc_bits;                   // LDR ADC reading (bits)
    int floor2_adc_mV;                     // LDR ADC reading (mV)
    int floor3_adc_bits;
    int floor3_adc_mV;

    while (1){
        
        // adc oneshot read for three LDRs
        adc_oneshot_read
        (adc2_handle, FLOOR1_LDR, &floor1_adc_bits);                // Read ADC bits (floor 1 LDR)
        
        adc_cali_raw_to_voltage
        (adc2_cali_chan_handle, floor1_adc_bits, &floor1_adc_mV);   // Convert to mV (floor 1 LDR)

        adc_oneshot_read
        (adc2_handle, FLOOR2_LDR, &floor2_adc_bits);                // Read ADC bits (floor 2 LDR)
        
        adc_cali_raw_to_voltage
        (adc2_cali_chan_handle, floor2_adc_bits, &floor2_adc_mV);   // Convert to mV (floor 2 LDR)
        
        adc_oneshot_read
        (adc2_handle, FLOOR3_LDR, &floor3_adc_bits);                // Read ADC bits (floor 3 LDR)
        
        adc_cali_raw_to_voltage
        (adc2_cali_chan_handle, floor3_adc_bits, &floor3_adc_mV);   // Convert to mV (floor 3 LDR)
    

        // initialize variables in relation to GPIO pin inputs (pushbuttons)
        floor1_callup = gpio_get_level(FLOOR1_CALLUP)==1;
        floor2_calldown = gpio_get_level(FLOOR2_CALLDOWN)==1;
        floor2_callup = gpio_get_level(FLOOR2_CALLUP)==1;
        floor3_calldown = gpio_get_level(FLOOR3_CALLDOWN)==1;
        floor1_select = gpio_get_level(FLOOR1_SELECT)==1;
        floor2_select = gpio_get_level(FLOOR2_SELECT)==1;
        floor3_select = gpio_get_level(FLOOR3_SELECT)==1;


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
    adc_oneshot_config_channel                          // Configure channel
    (adc2_handle, FLOOR1_LDR, &config);

    adc_oneshot_config_channel
    (adc2_handle, FLOOR2_LDR, &config);

    adc_oneshot_config_channel
    (adc2_handle, FLOOR3_LDR, &config);

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_2,
        .chan = FLOOR1_LDR,
        .atten = ADC_ATTEN,
        .bitwidth = BITWIDTH
    };
                                                      // Calibration config
    adc_cali_handle_t adc2_cali_chan_handle;            // Calibration handle
    adc_cali_create_scheme_curve_fitting                // Populate cal handle
    (&cali_config, &adc2_cali_chan_handle);
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
        .gpio_num       = LEDC_OUTPUT_IO,z
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
}


