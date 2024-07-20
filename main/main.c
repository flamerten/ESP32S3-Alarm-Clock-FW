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

#define PB_VALID_PRESS 50        // Debounce Time
#define PB_ACTIVE_HIGH 1
#define PB_ACTIVE_LOW 0

typedef struct PushButton
{   
    // Configurations

    gpio_num_t gpio_pin_no;              // e.g GPIO_NUM_0
    bool active_high;                    // 1 if active high, 0 if active low

    // System Readings
    bool last_reading;                   // Last GPIO Read
    uint32_t last_positive_read;         // Last time that 1 if active high, or 0 if active low was detected

} PushButton;


static const char *TAG = "main.c";


// Function Prototypes ---------------------/

static esp_err_t main_i2c_master_init(void);
static void init_gpios(void);

static int32_t ws2812_show_datetime(DateTime *time_now, led_strip_handle_t strip_handle);

static void pushbutton_configure(PushButton *push_button_config, gpio_num_t gpio_num, bool active_high);
static bool pushbutton_press_detected(PushButton *push_button_config);

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

    PushButton pb1_config;
    PushButton pb2_config;
    pushbutton_configure(&pb1_config, PCB_PB1, PB_ACTIVE_LOW);
    pushbutton_configure(&pb2_config, PCB_PB2, PB_ACTIVE_LOW);


    if(init_err != 0){
        ESP_LOGE(TAG,"Error in initialising");
    }
    
    bool curr_led_status;

    bool led0_on = false;
    bool led1_on = false;

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

        
        if(pushbutton_press_detected(&pb1_config)){
            led0_on = !led0_on;
            gpio_set_level(PCB_LED0, led0_on);
        }

        if(pushbutton_press_detected(&pb2_config)){
            led1_on = !led1_on;
            gpio_set_level(PCB_LED1, led1_on);
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
 * @brief Initialise the LEDs and the Mosfet controlling the Buck Converter. 
 * WARNING: The buck converter MUST be turned on for peripherals on the I2C Bus to work.
 * 
 */
static void init_gpios(void)
{   
    gpio_set_direction(PCB_LED0, GPIO_MODE_OUTPUT);
    gpio_set_direction(PCB_LED1, GPIO_MODE_OUTPUT);
    gpio_set_direction(PCB_BUCK_CTRL, GPIO_MODE_OUTPUT);
    
    gpio_set_pull_mode(PCB_LED0, GPIO_PULLDOWN_ONLY);
    gpio_set_pull_mode(PCB_LED1, GPIO_PULLDOWN_ONLY);

    gpio_set_level(PCB_BUCK_CTRL,1); //Turn on 3V7 for i2c bus

}

/**
 * @brief Show the time currently in the DateTime Struct 
 * 
 * @param time_now 
 * @param strip_handle 
 * @return int32_t 
 */
static int32_t ws2812_show_datetime(DateTime *time_now, led_strip_handle_t strip_handle)
{
    uint8_t hours   = time_now->hours;
    uint8_t minutes = time_now->minutes;
    int32_t err;

    err = ws2812_clear(strip_handle);

    ws2812_show_number(hours   / 10,  3,strip_handle);
    ws2812_show_number(hours   % 10,  2,strip_handle);

    ws2812_show_number(minutes / 10,  1,strip_handle);
    ws2812_show_number(minutes % 10,  0,strip_handle);

    err = ws2812_show(strip_handle);

    return err;
}

/**
 * @brief Fills up the Pushbutton struct with the gpio_num and whether it is an active high switch or active low switch.
 * Also initialises the GPIOs by setting it as input and whether its pullup or pulldown.
 * 
 * @param push_button_config    pointer
 * @param gpio_num              gpio_num_t 
 * @param active_high           1 if pb connected to vcc. 0 if pb connected to ground
 */
static void pushbutton_configure(PushButton *push_button_config, gpio_num_t gpio_num, bool active_high)
{
    push_button_config->active_high = active_high;
    push_button_config->gpio_pin_no = gpio_num;
    push_button_config->last_positive_read = esp_log_timestamp();

    gpio_set_direction(gpio_num, GPIO_MODE_INPUT);

    if(active_high)         gpio_set_pull_mode(gpio_num, GPIO_PULLDOWN_ONLY);
    else /*active low sw*/  gpio_set_pull_mode(gpio_num, GPIO_PULLUP_ONLY);
    
    push_button_config->last_reading = gpio_get_level(gpio_num);

}

/**
 * @brief Polling of pushbutton to check if it was called.
 * 
 * @param push_button_config Pointer
 * @return true     Button Press detected 
 * @return false    No Button Press detected
 */
static bool pushbutton_press_detected(PushButton *push_button_config)
{
    bool curr_level = gpio_get_level(push_button_config->gpio_pin_no);
    bool change_detected = (curr_level != push_button_config->last_reading);
    
    uint32_t curr_time = esp_log_timestamp();

    push_button_config->last_reading = curr_level;

    if      ( (push_button_config->active_high == 1) && (curr_level == 0) ) return 0;
    else if ( (push_button_config->active_high == 0) && (curr_level == 1) ) return 0;
    else if (  ((curr_time - push_button_config->last_positive_read)  > PB_VALID_PRESS) && change_detected)
    {   
        ESP_LOGI("PB", "%i Level: %i", push_button_config->gpio_pin_no, curr_level);

        push_button_config->last_positive_read = curr_time;

        return 1;
    }
    return 0;
}