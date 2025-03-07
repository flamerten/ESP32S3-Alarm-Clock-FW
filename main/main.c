#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "pcf8523/pcf8523.h"
#include "ws2812_leds/ws2812_leds.h"
#include "lsm6dsox/lsm6dsox_driver.h"

#include "PCB_Pin_Definitions.h"

//#define SET_RTC_TIME             //define in order to set default values to the PCF on bootup.
#define RTC_MINS     59
#define RTC_HOURS    12

#define PB_VALID_PRESS 200        // Debounce Time
#define PB_ACTIVE_HIGH 1
#define PB_ACTIVE_LOW  0

#define BATT_ADC_CHANNEL    ADC1_CHANNEL_5 //GPIO6 - PCB_BATT_ADC > GPIO_NUM_6

static const char *TAG = "main.c";


typedef struct PushButton
{   
    // Configurations
    gpio_num_t gpio_pin_no;              // e.g GPIO_NUM_0
    bool is_active_high;                    // 1 if active high, 0 if active low

    // System Readings
    bool last_reading;                   // Last GPIO Read
    uint32_t last_positive_read;         // Last time that gpio_read shows 1 if PB is set as active high, or 0 if PB is set as active low

} PushButton_t;

enum StateMachine {
    MODE_INACTIVE = 0,     //When LSM is inactive
    MODE_IDLE,
    MODE_SET_TIME,  
};

enum StateMachine curr_mode = MODE_IDLE;

// Function Prototypes ---------------------/

static esp_err_t main_i2c_master_init(void);
static void init_gpios(LSM_DriverConfig_t *imu_config);

static float get_batt_voltage(void);

static int32_t ws2812_show_datetime(Datetime *time_now, led_strip_handle_t strip_handle, bool animate);

static void pushbutton_configure(PushButton_t *push_button_config, gpio_num_t gpio_num, bool active_high);
static bool pushbutton_press_detected(PushButton_t *push_button_config);

static void change_mode(enum StateMachine new_mode);

// Objects --------------------------------/
static LSM_DriverConfig_t ClockIMU;
static led_strip_handle_t ClockLEDs;

PushButton_t ClockPbLeft;
PushButton_t ClockPbRight;

Datetime DatetimeShown;  // The Datetime shown on the LEDs
Datetime DatetimeNow;    // The Datetime extracted from the RTC

static esp_adc_cal_characteristics_t adc1_chars;

// Global Variables //HACK should be avoided?
uint32_t g_clock_hue = 0;    //Global variable - starting hue of clock. Increments during state change from INACTIVE to IDLE

void app_main(void)
{   
    //Might be good to set the log levels of different modules, where tag represents different modules
    /*
    esp_log_level_set(TAG, ESP_LOG_VERBOSE);

    ESP_LOGE(TAG,"Hello World - ERROR");
    ESP_LOGW(TAG,"Hello World - WARN");
    ESP_LOGI(TAG,"Hello World - INFO");
    ESP_LOGD(TAG,"Hello World - DEBUG");
    ESP_LOGV(TAG,"Hello World - VERBOSE");
    */
    
    printf("Hello World\n");

    //Initialisation -----------------------------

    init_gpios(&ClockIMU);
    main_i2c_master_init(); //NOTE: I2C init must come AFTER turning on the 3V7 Bus

    get_batt_voltage();

    int32_t init_err = 0;

    // I2C BUS ----------------------------------- 
    init_err = init_err | pcf8523_init(PCB_I2C_PORT);
    init_err = init_err | pcf8523_init(PCF8523_CTRL_7PF);

    init_err = init_err | lsm_init(&ClockIMU, PCB_I2C_PORT, PCB_LSM_SA);
    init_err = init_err | lsm_configure(&ClockIMU,
        LSM6DSOX_XL_ODR_208Hz, LSM6DSOX_2g,
        LSM6DSOX_GY_ODR_104Hz, LSM6DSOX_250dps);

    init_err = init_err | lsm_configure_activity(&ClockIMU);

    // LEDs -----------------------------------
    init_err = ws2812_init_leds(&ClockLEDs, PCB_NEOPIXEL_PIN, PCB_NEOPIXEL_COUNT);

    // Push Buttons -------------------------------
    pushbutton_configure(&ClockPbLeft, PCB_PB_LEFT, PB_ACTIVE_LOW);
    pushbutton_configure(&ClockPbRight, PCB_PB_RIGHT, PB_ACTIVE_LOW);

    if(init_err != 0){
        ESP_LOGE(TAG,"Error in initialising");
        while(1);
    }

    // Set up -----------------------------

    #ifdef SET_RTC_TIME
    DatetimeShown.minutes = RTC_MINS;
    DatetimeShown.hours = RTC_HOURS;

    DatetimeNow.minutes = RTC_MINS;
    DatetimeNow.hours = RTC_HOURS;

    pcf8523_adjust_datetime(&DatetimeShown);

    ESP_LOGI(TAG, "Using fw RTC Values: %i:%i", RTC_HOURS,RTC_MINS);
    #else  // SET_RTC_TIME is not defined in #define, use values stored in pcf
    pcf8523_get_datetime(&DatetimeNow); //Do the update in main loop
    DatetimeShown.show_hours = true;
    DatetimeShown.show_minutes = true;

    ESP_LOGI(TAG, "Using pcf time Values: %i:%i:%i", DatetimeNow.hours, DatetimeNow.minutes,DatetimeNow.seconds);
    #endif // SET_RTC_TIME
    //ws2812_show_datetime(&DatetimeNow,ClockLEDs);

    bool IDLE_digit_update_hours = 0;        // 0 is hours, 1 is updating minutes
    uint32_t last_blink_time = 0;
    bool blink_status = 0;                   // Used during MODE_IDLE to blink the digits. 1, digit is cleared. 0, digit is shown

    bool animate_flag = false; //Animate the clock when changing from MODE_INACTVITE to MODE_IDLE

    change_mode(MODE_INACTIVE);
    
    //Main Event Loop
    while(1)
    {   
        if(curr_mode == MODE_INACTIVE)   //Just check if need to switch back to idle mode
        {
            if(lsm_check_wake(&ClockIMU)==LSM_WAKE_STATUS_ACTIVE)
            {   
                g_clock_hue = (g_clock_hue + 20)%360;
                animate_flag = true;
                change_mode(MODE_IDLE);
            }
            else{
                vTaskDelay(pdMS_TO_TICKS(20)); //Prevent watchdog error being thrown thrown
            }

        }
        else if(curr_mode == MODE_IDLE)  // Read from PCF, update LED clock if needed. Check for Either PB press to change to MODE_SET_TIME
        {
            pcf8523_get_datetime(&DatetimeNow);

            //Only update the clock if there is a change in time, OR animate flag is raised -> during state change
            if( animate_flag | 
                (DatetimeShown.minutes != DatetimeNow.minutes) | 
                (DatetimeShown.hours != DatetimeNow.hours) | 
                (DatetimeShown.seconds != DatetimeNow.seconds) )
            {
                ESP_LOGI(TAG,"Timing update from PCF: %i:%i:%i",DatetimeNow.hours,DatetimeNow.minutes,DatetimeNow.seconds);
                DatetimeShown.hours   = DatetimeNow.hours;
                DatetimeShown.minutes = DatetimeNow.minutes;
                DatetimeShown.seconds = DatetimeNow.seconds;

                DatetimeShown.show_hours = true;
                DatetimeShown.show_minutes = true;
                ws2812_show_datetime(&DatetimeShown,ClockLEDs,animate_flag);

                animate_flag = false;
            }

            //Poll alarm clock buttons to change state machine
            if(pushbutton_press_detected(&ClockPbLeft) || pushbutton_press_detected(&ClockPbRight)) //Press any button to switch to set time mode
            { 
                DatetimeShown.hours   = DatetimeNow.hours;
                DatetimeShown.minutes = DatetimeNow.minutes;

                DatetimeNow.last_pcf_update = esp_log_timestamp();
                change_mode(MODE_SET_TIME);
            }
            else if(lsm_check_wake(&ClockIMU)==LSM_WAKE_STATUS_INACTIVE)               //Interrupt detected + inactivty detected
            {   
                DatetimeShown.show_hours = false;
                DatetimeShown.show_minutes = false;
                ws2812_show_datetime(&DatetimeShown,ClockLEDs,false);

                change_mode(MODE_INACTIVE);
                
            }        
        }
        else if(curr_mode == MODE_SET_TIME){
            //Update DatetimeNow, and from there play with DatetimeShown. Then at the end write datetimenow to PCF.

            uint32_t curr_time = esp_log_timestamp();

            if(pushbutton_press_detected(&ClockPbLeft))      //Switch to hours/minutes
            {
                IDLE_digit_update_hours = !IDLE_digit_update_hours; 
                DatetimeNow.last_pcf_update = curr_time;

                ESP_LOGI(TAG,"Changing digit update hours %i",IDLE_digit_update_hours);
            }
            else if(pushbutton_press_detected(&ClockPbRight)) //Increment hours/minutes
            {
                if(IDLE_digit_update_hours){
                    if(DatetimeNow.hours >= 23)   DatetimeNow.hours = 0;
                    else                          DatetimeNow.hours = DatetimeNow.hours + 1;
                }
                else{
                    if(DatetimeNow.minutes >= 59)  DatetimeNow.minutes = 0;
                    else                           DatetimeNow.minutes = DatetimeNow.minutes + 1;
                }

                // Don't blink during digit updates
                blink_status = 0;
                last_blink_time = curr_time;

                DatetimeNow.last_pcf_update = curr_time;
                ESP_LOGI(TAG,"Time Updated to %i:%i",DatetimeNow.hours,DatetimeNow.minutes);
            }

            //Show result on the clock, and blink the digit
            DatetimeShown.minutes = DatetimeNow.minutes;
            DatetimeShown.hours = DatetimeNow.hours;

            if(blink_status){
                if(IDLE_digit_update_hours) DatetimeShown.show_hours = false;
                else                        DatetimeShown.show_minutes = false;
            }
            else{
                DatetimeShown.show_hours = true;
                DatetimeShown.show_minutes = true;
            }
            
            if(curr_time - last_blink_time >= 500){ //Blink the digit every 0.5s
                blink_status = !blink_status;
                last_blink_time = curr_time;
            }

            if(curr_time - DatetimeNow.last_pcf_update >= 5000){ //If no change for 5s, switch back to idle mode
                DatetimeShown.show_hours = true;
                DatetimeShown.show_minutes = true;
                DatetimeNow.seconds = 0; //Reset seconds to 0

                pcf8523_adjust_datetime(&DatetimeNow); //Write new val to PCF
                change_mode(MODE_IDLE);
            }

            ws2812_show_datetime(&DatetimeShown,ClockLEDs,false);
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
static void init_gpios(LSM_DriverConfig_t *imu_config)
{   
    //LEDS
    gpio_set_direction(PCB_LED0, GPIO_MODE_OUTPUT);
    gpio_set_direction(PCB_LED1, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(PCB_LED0, GPIO_PULLDOWN_ONLY);
    gpio_set_pull_mode(PCB_LED1, GPIO_PULLDOWN_ONLY);

    //Buck converter (3V7). //NOTE: MUST BE TURNED ON. for the I2C bus to function properly 
    gpio_set_direction(PCB_BUCK_CTRL, GPIO_MODE_OUTPUT);
    gpio_set_level(PCB_BUCK_CTRL,1);

    //LSM Interrupt Pin
    gpio_set_direction(PCB_LSM_INT1, GPIO_MODE_INPUT);

    //Analog Read Configuration //HACK this ver of ADC reading is depreceated
    //Reference used: https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32s3/api-reference/peripherals/adc.html
    // - consider changing it to v5 espidf https://docs.espressif.com/projects/esp-idf/en/v5.2.2/esp32s3/api-reference/peripherals/adc_continuous.html  
    int adc_atten =  ADC_ATTEN_DB_12; //0 mV ~ 3100 mV 
    int adc_calib_scheme = ESP_ADC_CAL_VAL_EFUSE_TP_FIT; //For esp target ESP32S3
    
    esp_err_t ret = esp_adc_cal_check_efuse(adc_calib_scheme);
    if (ret == ESP_ERR_NOT_SUPPORTED)         ESP_LOGW("ADC", "Calibration scheme not supported, skip software calibration");
    else if (ret == ESP_ERR_INVALID_VERSION)  ESP_LOGW("ADC", "eFuse not burnt, skip software calibration");
    else if (ret == ESP_ERR_INVALID_ARG)      ESP_LOGE("ADC", "Invalid arg");
    else{
        esp_adc_cal_characterize(ADC_UNIT_1, adc_atten, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);
    }                                 

    ESP_ERROR_CHECK( adc1_config_channel_atten(BATT_ADC_CHANNEL, adc_atten) );
}

/**
 * @brief Get battery voltage of the PCB, by reading the ADC of GPIO_NUM_6 and multiplying by 2
 * 
 * @return float 
 */
static float get_batt_voltage(void)
{   
    int adc_raw = adc1_get_raw(BATT_ADC_CHANNEL);
    uint32_t adc_voltage = esp_adc_cal_raw_to_voltage(adc_raw, &adc1_chars); //Returns in mv
    float batt_voltage = adc_voltage/1000.0 * 2;                             //Convert to V, multiply by 2

    ESP_LOGI(TAG,"Batt Voltage %.2fmv",batt_voltage);
    return batt_voltage;
}

/**
 * @brief Show the time currently in the Datetime Struct. 
 * 
 * @param time_now 
 * @param strip_handle
 * @param animate true to animate, else, simply show in one shot
 * @return int32_t 
 */
static int32_t ws2812_show_datetime(Datetime *time_now, led_strip_handle_t strip_handle, bool animate)
{
    uint8_t hours   = time_now->hours;
    uint8_t minutes = time_now->minutes;
    int32_t err;

    int32_t buffer = 0;

    //Write to buffer
    if(time_now->show_hours == false){
        buffer = ws2812_update_buffer(255,3,buffer);
        buffer = ws2812_update_buffer(255,2,buffer);
    }
    else{
        buffer = ws2812_update_buffer(hours / 10, 3, buffer);
        buffer = ws2812_update_buffer(hours % 10, 2, buffer);
    }

    if(time_now->show_minutes == false){
        buffer = ws2812_update_buffer(255, 1, buffer);
        buffer = ws2812_update_buffer(255, 0, buffer);
    }
    else{
        buffer = ws2812_update_buffer(minutes / 10, 1, buffer);
        buffer = ws2812_update_buffer(minutes % 10, 0, buffer);
    }

    uint16_t sat = 255;
    uint16_t val = 5;
    uint32_t hue = g_clock_hue;

    err = ws2812_clear(strip_handle);

    for(int idx = 0; idx < PCB_NEOPIXEL_COUNT; idx++)
    {
        if(buffer & 1){
            ws2812_set_pixel_hsv(strip_handle,idx,hue,sat,val);
        }
        else{
            ws2812_set_pixel_hsv(strip_handle,idx,hue,sat,0); //Set to 0 brightness
        }
        
        hue = hue + 5;           //So that during blinking there is no colour change
        buffer = (buffer >> 1);
        
        if(animate){
            vTaskDelay(pdMS_TO_TICKS(40));
            ws2812_show(strip_handle);
        }
    }


    if( (time_now-> show_hours == false) & (time_now->show_minutes == false)) //Turn off the seperator too
    {
        err = err | ws2812_set_pixel_hsv(strip_handle,PCB_CLOCKIDX_SEP,hue,0,0); 
    }
    
    //Else we toggle the seperator depending on the seconds
    else if(time_now->seconds%2) //White
    {
        ws2812_set_pixel_hsv(strip_handle,PCB_CLOCKIDX_SEP,0,0,5); //white 
    }
    else                         //Off
    {
        ws2812_set_pixel_hsv(strip_handle,PCB_CLOCKIDX_SEP,0,0,0); //white
    }

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
static void pushbutton_configure(PushButton_t *push_button_config, gpio_num_t gpio_num, bool active_high)
{
    push_button_config->is_active_high = active_high;
    push_button_config->gpio_pin_no = gpio_num;
    push_button_config->last_positive_read = esp_log_timestamp();

    gpio_set_direction(gpio_num, GPIO_MODE_INPUT);

    if(active_high)         gpio_set_pull_mode(gpio_num, GPIO_PULLDOWN_ONLY);
    else /*active low sw*/  gpio_set_pull_mode(gpio_num, GPIO_PULLUP_ONLY);
    
    push_button_config->last_reading = gpio_get_level(gpio_num);

}

/**
 * @brief Polling of pushbutton to check if it has been pressed
 * 
 * @param push_button_config Pointer
 * @return true     Button Press detected 
 * @return false    No Button Press detected
 */
static bool pushbutton_press_detected(PushButton_t *push_button_config)
{   
    bool curr_level = gpio_get_level(push_button_config->gpio_pin_no);
    //bool change_detected = (curr_level != push_button_config->last_reading);
    //Can be used to prevent holding from being detected as multipress
    
    uint32_t curr_time = esp_log_timestamp();

    push_button_config->last_reading = curr_level;

    if      ( (push_button_config->is_active_high == PB_ACTIVE_HIGH) && (curr_level == 0) ) return 0; //Not being pressed
    else if ( (push_button_config->is_active_high == PB_ACTIVE_LOW)  && (curr_level == 1) ) return 0; //Not being pressed
    else if ( ( (curr_time - push_button_config->last_positive_read)  > PB_VALID_PRESS) /*&& change_detected*/)
    {   
        ESP_LOGD(TAG, "PB GPIO %i Level: %i pressed", push_button_config->gpio_pin_no, curr_level);

        push_button_config->last_positive_read = curr_time;

        return 1;
    }
    return 0;
}

/**
 * @brief Change state machine mode
 * 
 * @param new_mode must be from enum StateMachine
 */
static void change_mode(enum StateMachine new_mode)
{
    curr_mode = new_mode;
    if(new_mode == MODE_IDLE)          ESP_LOGI("StateMachine", "MODE_IDLE");
    else if(new_mode == MODE_SET_TIME) ESP_LOGI("StateMachine", "MODE_SET_TIME");
    else if(new_mode == MODE_INACTIVE) ESP_LOGI("StateMachine", "MODE_INACTIVE");
    
}