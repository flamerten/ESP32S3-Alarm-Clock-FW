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
static void init_gpios(void);


void app_main(void)
{   
    ESP_LOGI(TAG,"Hello World");
    printf("Hello World");

    init_gpios();

    main_i2c_master_init();

    test_pcf();

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

    i2c_param_config(I2C_PORT, &conf);

    return i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}

static void test_pcf(void)
{
    pcf8523_init(I2C_PORT);

    DateTime datetime_obj;
    pcf_8523_timenow(&datetime_obj);

    datetime_obj.seconds = 0;
    datetime_obj.minutes = RTC_MINS;
    datetime_obj.hours = RTC_HOURS;

    print_datetime(&datetime_obj);
    pcf8523_adjustTime(&datetime_obj);

    vTaskDelay(5000/portTICK_PERIOD_MS);

    pcf_8523_timenow(&datetime_obj);
    print_datetime(&datetime_obj);

}

/**
 * @brief Initialise the LEDs and the Mosfet controlling the Buck Converter. 
 * WARNING: The buck converter MUST be turned on for peripherals on the I2C Bus to work.
 * 
 */
static void init_gpios(void)
{   
    gpio_set_direction(LED0,GPIO_MODE_OUTPUT);
    gpio_set_direction(LED1,GPIO_MODE_OUTPUT);
    gpio_set_direction(BUCK,GPIO_MODE_OUTPUT);

    gpio_set_pull_mode(LED0,GPIO_PULLDOWN_ENABLE);
    gpio_set_pull_mode(LED1,GPIO_PULLDOWN_ENABLE);

    gpio_set_level(BUCK,1); //Turn on 3V7 for i2c bus

}