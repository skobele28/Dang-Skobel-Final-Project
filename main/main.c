#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include <sys/time.h>
#include "../components/LCD/esp-idf-lib__esp_idf_lib_helpers/esp_idf_lib_helpers.h"
#include "../components/LCD/esp-idf-lib__hd44780/hd44780.h"
#include <inttypes.h>
#include <stdio.h>
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/ledc.h"

//Global variables
//Pins declaration
#define Elevator1           
#define Elevator2  
#define Elevator3
#define Floor1_call
#define Floor2_callup
#define Floor2_calldown
#define Floor3_call
#define am2302
#define buzzer
//LCD pins are inside the LCD init function

//LEDC configuration
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO      (16)        //GPIO for the LEDC .
#define LEDC_CHANNEL        LEDC_CHANNEL_0
#define LEDC_DUTY_RES       LEDC_TIMER_13_BIT 
#define LEDC_FREQUENCY      (50) // Frequency in Hertz.
#define go_up               ()
#define go_down             ()

//LCD structure, including GPIOs
hd44780_t lcd = {
    .write_cb = NULL,
    .font = HD44780_FONT_5X8,
    .lines = 2,
    .pins = {
        .rs = GPIO_NUM_8,                           // GPIO for Register Select
        .e  = GPIO_NUM_3,                           // GPIO for enable
        .d4 = GPIO_NUM_9,                           // GPIO for data 4
        .d5 = GPIO_NUM_10,                          // GPIO for data 5
        .d6 = GPIO_NUM_11,                          // GPIO for data 6
        .d7 = GPIO_NUM_12,                          // GPIO for data 7
        .bl = HD44780_NOT_USED
    }
};

//Global boolean values

//function prototypes
static void pinConfig(void);  
static void ledc_initialize(void);
static void input_task(void);
static void servo_task(void);
static void floor_logic(void);

void app_main(void)
{

}


void pinConfig(void){
    //Reset pins
    gpio_reset_pin(Elevator1);
    gpio_reset_pin(Elevator2);
    gpio_reset_pin(Elevator3);
    gpio_reset_pin(Floor1_call);
    gpio_reset_pin(Floor2_calldown);
    gpio_reset_pin(Floor2_callup);
    gpio_reset_pin(Floor3_call);
    gpio_reset_pin(am2302);
    gpio_reset_pin(buzzer);

    //Set directions
    gpio_set_direction(Elevator1);
    gpio_set_direction(Elevator2);
    gpio_set_direction(Elevator3);
    gpio_set_direction(Floor1_call);
    gpio_set_direction(Floor2_calldown);
    gpio_set_direction(Floor2_callup);
    gpio_set_direction(Floor3_call);
    gpio_set_direction(am2302);
    gpio_set_direction(buzzer);

    //Configure pullup/down
    gpio_pulldown_en(Elevator1);
    gpio_pulldown_en(Elevator2);
    gpio_pulldown_en(Elevator3);
    gpio_pulldown_en(Floor1_call);
    gpio_pulldown_en(Floor2_calldown);
    gpio_pulldown_en(Floor2_callup);
    gpio_pulldown_en(Floor3_call);
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
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
}