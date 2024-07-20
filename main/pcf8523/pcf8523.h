#ifndef PCF8523_DRV

#define PCF8523_DRV

#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/i2c.h"

// Control Registers ------------------------------------

#define PCF8523_ADDRESS 0x68       ///< I2C address for PCF8523
#define PCF8523_CLKOUTCONTROL 0x0F ///< Timer and CLKOUT control register
#define PCF8523_CONTROL_1 0x00     ///< Control and status register 1
#define PCF8523_CONTROL_2 0x01     ///< Control and status register 2
#define PCF8523_CONTROL_3 0x02     ///< Control and status register 3
#define PCF8523_TIMER_B_FRCTL 0x12 ///< Timer B source clock frequency control
#define PCF8523_TIMER_B_VALUE 0x13 ///< Timer B value (number clock periods)
#define PCF8523_OFFSET 0x0E        ///< Offset register
#define PCF8523_STATUSREG 0x03     ///< Status register

// Commands ------------------------------------

#define PCF8523_RST 0x58  

// Configurations ------------------------------------

#define I2C_MASTER_TIMEOUT_MS     1000     //Time to wait before ESP_ERR_TIMEOUT is issued when trying to read/write to/from I2C bus
#define PCF8523_INIT_FALSE_ERR    -2       //Error code when any function has been called before calling pcf8523_init

#define PCF8523_CTRL_7PF          0        //Quartz Crystal Load Capacitance (7pF def)
#define PCF8523_CTRL_12PF5        1        //Quartz Crystal Load Capacitance (12.5pF)
#define PCF8523_CTRL_12HR_MODE    1        //12 Hour Mode
#define PCF8523_CTRL_24HR_MODE    1        //24 Hour Mode Default



/**
 * @brief Struct to hold date variables
 * 
 */
typedef struct DateTime
{
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;

    uint8_t day;
    uint8_t weekday;

    uint8_t month;
    uint32_t year;

    uint32_t last_pcf_update;   //esp_log_timestamp to check when the struct was last updated
} DateTime;

/**
 * @brief Struct that is passed into platform_read and platform write
 * 
 */
typedef struct
{
    i2c_port_t i2c_port_num;
    uint8_t i2c_addr;
}I2C_Sensor;

typedef struct
{
    I2C_Sensor i2c_sensor_handle;
}PCF8523_Config;

//PCF

static PCF8523_Config pcf8523_config;
static uint8_t pcf_init_called = 0;

int32_t pcf8523_init(i2c_port_t i2c_port_num);
int32_t pcf8523_timenow(DateTime* time_now);
int32_t pcf8523_adjustTime(DateTime* time_now);
int32_t pcf8523_configure_ctrl1(bool capacitance, bool time_mode);

//Utility

void print_datetime(DateTime *datetime);

#endif //PCF8523_DRV