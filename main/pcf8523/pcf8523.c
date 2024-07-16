/**
 * @file pcf8523.c
 * @author Samuel Yow
 * @brief .c file for the PCF8523 driver. Created for ESP-IDF v5.2.1
 * 
 * Source
 *  - https://github.com/adafruit/RTClib/blob/master/src/RTClib.h
 *  - https://github.com/adafruit/RTClib/blob/master/src/RTC_PCF8523.cpp
 */

#include <stdio.h>
#include "pcf8523.h"


char* DaysOfTheWeek[] = {"Sun", "Mon", "Tue", "Wed","Thu","Fri", "Sat"};


// Platform Independent Commands --------------------------------/

/**
 * @brief    Handle i2c errors in ESP-iDF using esp_log
 * 
 * @param TAG          tag for esp_log
 * @param err          esp_err_t
 * @return int32_t     0 if no error, else return err code in esp_err_t
 */
static int32_t handle_esp_err(char *TAG,esp_err_t err)

{
    if(err == ESP_OK)                       return 0;
    else if(err == ESP_ERR_INVALID_ARG)     ESP_LOGE(TAG,"Parameter Error");
    else if(err == ESP_ERR_INVALID_ARG)     ESP_LOGE(TAG,"Sending command error, slave hasn’t ACK the transfer.");
    else if(err == ESP_ERR_INVALID_STATE)   ESP_LOGE(TAG,"I2C driver not installed or not in master mode");
    else if(err == ESP_ERR_TIMEOUT)         ESP_LOGE(TAG,"Operation timeout because the bus is busy");
    else                                    ESP_LOGE(TAG,"Unkown Error %d",err);

    return 1;
}

/**
 * @brief    Write generic device register for ESP-IDF based on PID
 * 
 * @param handle       I2C_Sensor - for setting i2c addr and port number
 * @param Reg          register to write
 * @param Bufp         pointer to data to write in register reg
 * @param len          number of consecutive register to write
 * @return int32_t     0 if ok, else return esp_err_t
 */
static int32_t platform_write(void *handle, uint8_t Reg, const uint8_t *Bufp, uint16_t len)
{   
    I2C_Sensor *sensor_config = (I2C_Sensor *)handle;
    uint8_t sensor_addr = sensor_config->i2c_addr;
    i2c_port_t i2c_port_number = sensor_config->i2c_port_num;

    //Create a new buffer, message that includes register to write to
    uint8_t msg[len + 1];
    msg[0] = Reg;
    memcpy(&msg[1],&Bufp[0],len * sizeof(Reg));

    esp_err_t err = i2c_master_write_to_device(
        i2c_port_number,
        sensor_addr,
        msg,
        len + 1,
        I2C_MASTER_TIMEOUT_MS/portTICK_PERIOD_MS    
    );

    if(err == 0)   return 0;
    else           return handle_esp_err("platform_write",err);
}

/**
 * @brief    Read generic device register for ESP-IDF based on PID 
 * 
 * @param handle       I2C_Sensor - for setting i2c addr and port number 
 * @param Reg          register to read
 * @param Bufp         pointer to buffer that stores the data read
 * @param len          number of consecutive register to read
 * @return int32_t     0 if okay, else return esp_err_t
 */
static int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len)
{   
    I2C_Sensor *sensor_config = (I2C_Sensor *)handle;
    uint8_t sensor_addr = sensor_config->i2c_addr;
    i2c_port_t i2c_port_number = sensor_config->i2c_port_num;

    esp_err_t err = i2c_master_write_read_device(
        i2c_port_number,
        sensor_addr, 
        &Reg,
        1,
        Bufp,
        len,
        I2C_MASTER_TIMEOUT_MS/portTICK_PERIOD_MS    
    );

    if(err == 0)   return 0;
    else           return handle_esp_err("platform_read",err);
}

// Utility Commands --------------------------------/

static uint8_t bcd2bin(uint8_t val) 
{
    return val - 6 * (val >> 4); 
}

static uint8_t bin2bcd(uint8_t val) 
{ 
    return val + 6 * (val / 10); 
}


// Gloabal PCF Functions ---------------------------------------/

int32_t pcf8523_init(PCF8523_Config *sensor_cfg,i2c_port_t i2c_port_num)
{
    
    //Configure I2C Sensor
    sensor_cfg->i2c_sensor_handle.i2c_addr = PCF8523_ADDRESS;
    sensor_cfg->i2c_sensor_handle.i2c_port_num = i2c_port_num;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( PCF8523_ADDRESS << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
    i2c_master_stop(cmd);   

    uint32_t res = i2c_master_cmd_begin(i2c_port_num, cmd, I2C_MASTER_TIMEOUT_MS/portTICK_PERIOD_MS);

    i2c_cmd_link_delete(cmd);

    if(!res) ESP_LOGI("pcf8523_init","pcf init ok");

    return handle_esp_err("pcf8523_init",res);
    
}

/**
 * @brief              Write the current time on the RTC into the Datetime object
 * 
 * @param time_now     Datetime
 * @param sensor_cfg   sensor
 * @return int32_t     0 if no errors
 */
int32_t pcf_8523_timenow(Datetime* time_now, PCF8523_Config* sensor_cfg)
{
    uint8_t buffer[7];
    uint8_t res = platform_read(
        &(sensor_cfg->i2c_sensor_handle),
        PCF8523_STATUSREG,
        buffer,
        7);

    time_now->year    = bcd2bin(buffer[6]) + 2000U;
    time_now->month   = bcd2bin(buffer[5]);
    time_now->weekday = bcd2bin(buffer[4]);
    time_now->day     = bcd2bin(buffer[3]);
    time_now->hours   = bcd2bin(buffer[2]);
    time_now->minutes = bcd2bin(buffer[1]);
    time_now->seconds = bcd2bin(buffer[0] & 0x7F); //remove 7th bit (OS stop Flag)

    return res;

}

/**
 * @brief              Write a new time into the RTC using the Datetime object
 * 
 * @param time_now     Datetime*
 * @param sensor_cfg   sensor
 * @return int32_t     0 if no errors
 */
int32_t pcf8523_adjustTime(Datetime* time_now, PCF8523_Config* sensor_cfg)
{
    uint8_t buffer[7] = {bin2bcd(time_now->seconds),
                         bin2bcd(time_now->minutes),
                         bin2bcd(time_now->hours),
                         bin2bcd(time_now->day),
                         bin2bcd(0), // skip weekdays
                         bin2bcd(time_now->month),
                         bin2bcd(time_now->year - 2000U)};
    
    int32_t res = platform_write(
        &(sensor_cfg->i2c_sensor_handle),
        PCF8523_STATUSREG,
        buffer,
        7);

    return res;      
}

void print_datetime(Datetime *datetime)
{
    printf("Date> %i-%i-%lu ",
        datetime->day,
        datetime->month,
        datetime->year);
    printf(" %s ",DaysOfTheWeek[datetime->weekday]);
    printf(" Time> %i:%i:%i \n",
        datetime->hours,
        datetime->minutes,
        datetime->seconds);
}




