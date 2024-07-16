#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "pcf8523/pcf8523.h"
#include "ws2812_leds/ws2812_leds.h"

#include "PCB_Pin_Definitions.h"


#define RTC_MINS     59
#define RTC_HOURS    12


static const char *TAG = "main.c";


// Function Prototypes ---------------------/

static esp_err_t main_i2c_master_init(void);
static void init_gpios(void);

static int32_t ws2812_show_datetime(DateTime *time_now, led_strip_handle_t strip_handle);

void app_main(void)
{   
    ESP_LOGI(TAG,"Hello World");
    printf("Hello World\n");

    //Initialisation

    init_gpios();
    main_i2c_master_init();

    int32_t init_err = 0;

    init_err = pcf8523_init(PCB_I2C_PORT);
    init_err = pcf8523_configure_ctrl1(PCF8523_CTRL_7PF, PCF8523_CTRL_12HR_MODE);

    DateTime datetime_register_shown;
    DateTime datetime_register_now;

    datetime_register_shown.minutes = RTC_MINS;
    datetime_register_shown.hours = RTC_HOURS;

    datetime_register_now.minutes = RTC_MINS;
    datetime_register_now.hours = RTC_HOURS;

    init_err = pcf8523_adjustTime(&datetime_register_shown);

    led_strip_handle_t clock_leds;
    init_err = ws2812_init_leds(&clock_leds, PCB_NEOPIXEL_PIN, PCB_NEOPIXEL_COUNT);
    init_err = ws2812_show_datetime(&datetime_register_now,clock_leds);

    if(init_err != 0){
        ESP_LOGE(TAG,"Error in initialising");
    }

    //Main Event Loop
    while(1)
    {
        pcf_8523_timenow(&datetime_register_now);
        
        if( (datetime_register_shown.minutes != datetime_register_now.minutes) ||
            (datetime_register_shown.hours != datetime_register_now.hours) )
        {
            //Update Registers
            datetime_register_shown.minutes = datetime_register_now.minutes;
            datetime_register_shown.hours = datetime_register_now.hours;

            printf("New Timing: %i:%i \n",datetime_register_shown.hours,datetime_register_shown.minutes);

            ws2812_show_datetime(&datetime_register_shown,clock_leds);
        }

        else{
            vTaskDelay(200/portTICK_PERIOD_MS);
        }
    }



}

// Functions ---------------------------------

static esp_err_t main_i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PCB_I2C_SDA,
        .scl_io_num = PCB_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = 100000
    };

    i2c_param_config(PCB_I2C_PORT, &conf);

    return i2c_driver_install(PCB_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}

/**
 * @brief Initialise the LEDs, Push Buttons and the Mosfet controlling the Buck Converter. 
 * WARNING: The buck converter MUST be turned on for peripherals on the I2C Bus to work.
 * 
 */
static void init_gpios(void)
{   
    gpio_set_direction(PCB_LED0, GPIO_MODE_OUTPUT);
    gpio_set_direction(PCB_LED1, GPIO_MODE_OUTPUT);
    gpio_set_direction(PCB_BUCK_CTRL, GPIO_MODE_OUTPUT);

    gpio_set_pull_mode(PCB_LED0, GPIO_PULLDOWN_ENABLE);
    gpio_set_pull_mode(PCB_LED1, GPIO_PULLDOWN_ENABLE);

    gpio_set_level(PCB_BUCK_CTRL,1); //Turn on 3V7 for i2c bus

}

/**
 * @brief Show the time currently in the DateTime Struct 
 * 
 * @param time_now 
 * @param strip_handle 
 * @return int32_t 
 */
int32_t ws2812_show_datetime(DateTime *time_now, led_strip_handle_t strip_handle)
{
    uint8_t hours   = time_now->hours;
    uint8_t minutes = time_now->minutes;
    int32_t err;

    err = ws2812_clear(strip_handle);

    ws2812_show_number(hours   / 10,  3,strip_handle);
    ws2812_show_number(hours   % 10,  2,strip_handle);

    ws2812_show_number(minutes / 10,1,strip_handle);
    ws2812_show_number(minutes % 10,0,strip_handle);

    err = ws2812_show(strip_handle);

    return err;
}