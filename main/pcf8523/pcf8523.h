#ifndef PCF8523_DRV
#define PCF8523_DRV

#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/i2c.h"

#include "TimedateTypes.h"

#define PCF8523_ADDRESS 0x68       ///< I2C address for PCF8523
#define PCF8523_CLKOUTCONTROL 0x0F ///< Timer and CLKOUT control register
#define PCF8523_CONTROL_1 0x00     ///< Control and status register 1
#define PCF8523_CONTROL_2 0x01     ///< Control and status register 2
#define PCF8523_CONTROL_3 0x02     ///< Control and status register 3
#define PCF8523_TIMER_B_FRCTL 0x12 ///< Timer B source clock frequency control
#define PCF8523_TIMER_B_VALUE 0x13 ///< Timer B value (number clock periods)
#define PCF8523_OFFSET 0x0E        ///< Offset register
#define PCF8523_STATUSREG 0x03     ///< Status register

#define PCF8523_RST 0x58

#define I2C_MASTER_TIMEOUT_MS 1000


/*
typedef struct
{
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;

    uint8_t day;
    uint8_t weekday;

    uint8_t month;
    uint32_t year;
}Datetime;
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

int32_t pcf8523_init(PCF8523_Config *sensor_cfg,i2c_port_t i2c_port_num);
int32_t pcf_8523_timenow(Datetime* time_now, PCF8523_Config* sensor_cfg);
int32_t pcf8523_adjustTime(Datetime* time_now, PCF8523_Config* sensor_cfg);

//Utility

void print_datetime(Datetime *datetime);

#endif //PCF8523_DRV