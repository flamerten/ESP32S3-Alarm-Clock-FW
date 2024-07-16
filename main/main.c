#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pcf8523/pcf8523.h"
#include "PinDef.h"

#define RTC_MINS     13
#define RTC_HOURS    10


static const char *TAG = "main.c";


// Function Prototypes ---------------------/

static esp_err_t main_i2c_master_init(void);
static void test_pcf(void);

void app_main(void)
{   
    ESP_LOGI(TAG,"Hello World");
    printf("Hello World");

    main_i2c_master_init();

    test_pcf();

}

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

    i2c_param_config(I2C_PORT, &conf);

    return i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}

static void test_pcf(void)
{
    PCF8523_Config rtc_config;
    pcf8523_init(&rtc_config,I2C_PORT);

    Datetime datetime_obj;
    pcf_8523_timenow(&datetime_obj, &rtc_config);

    datetime_obj.seconds = 0;
    datetime_obj.minutes = RTC_MINS;
    datetime_obj.hours = RTC_HOURS;

    print_datetime(&datetime_obj);
    pcf8523_adjustTime(&datetime_obj,&rtc_config);

    vTaskDelay(5000/portTICK_PERIOD_MS);

    pcf_8523_timenow(&datetime_obj, &rtc_config);
    print_datetime(&datetime_obj);

}