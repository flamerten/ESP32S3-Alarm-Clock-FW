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



/**
 * @brief Struct to hold date variables.
 * 
 */
typedef struct Datetime
{
    uint8_t seconds;          // 0 - 59 
    uint8_t minutes;          // 0 - 59
    uint8_t hours;            // 0 - 23 (Default to 24 hour mode)

    uint8_t day;              // 1 - 31
    uint8_t weekday;          // 0 - 6

    uint8_t month;            // 1 - 12
    uint32_t year;            // 0 - 99

    uint32_t last_pcf_update;   //esp_log_timestamp to check when the struct was last updated

    bool show_hours;           //Show on clock?
    bool show_minutes;         //Show on clock?

} Datetime;

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

int32_t pcf8523_init(i2c_port_t i2c_port_num);
int32_t pcf8523_get_datetime(Datetime* time_now);
int32_t pcf8523_adjust_datetime(Datetime* time_now);
int32_t pcf8523_set_capacitance(bool capacitance);

//Utility

void print_datetime(Datetime *datetime);

#endif //PCF8523_DRV