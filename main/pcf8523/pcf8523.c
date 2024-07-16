/**
 * @file pcf8523.c
 * @author Samuel Yow
 * @brief .c file for the PCF8523 driver. Created for ESP-IDF v5.2.1
 * 
 * Source
 *  - https://github.com/adafruit/RTClib/blob/master/src/RTClib.h
 *  - https://github.com/adafruit/RTClib/blob/master/src/RTC_PCF8523.cpp
 * 
 * The goal is to create a platform independent driver where the user simply has to edit certain functions and get the 
 * PCF driver working in any embedded environment
 * 
 */

#include <stdio.h>
#include "pcf8523.h"


char* DaysOfTheWeek[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

// Platform Independent Commands --------------------------------/

/**
 * @brief    Handle i2c errors in ESP-iDF using esp_log
 * 
 * @param TAG          tag for esp_log
 * @param err          esp_err_t
 * @return int32_t     0 if no error, else return err code in esp_err_t
 */
static int32_t handle_err(char *TAG, int32_t err)

{
    if(err == ESP_OK)                       return 0;
    else if(err == PCF8523_INIT_FALSE_ERR)  ESP_LOGE(TAG,"Need to initialize PCF first!");
    else if(err == ESP_ERR_INVALID_ARG)     ESP_LOGE(TAG,"Parameter Error");
    else if(err == ESP_ERR_INVALID_ARG)     ESP_LOGE(TAG,"Sending command error, slave hasn’t ACK the transfer.");
    else if(err == ESP_ERR_INVALID_STATE)   ESP_LOGE(TAG,"I2C driver not installed or not in master mode");
    else if(err == ESP_ERR_TIMEOUT)         ESP_LOGE(TAG,"Operation timeout because the bus is busy");
    else                                    ESP_LOGE(TAG,"Unkown Error %d", (int)err); //Print in hex format 

    return 1;
}

/**
 * @brief    Write generic device register for ESP-IDF based on PID
 * 
 * @param handle       I2C_Sensor - for setting i2c addr and port number. Can be configured to any other handle for other platforms
 * @param Reg          register to write
 * @param Bufp         pointer to data to write in register reg
 * @param len          number of consecutive register to write
 * @return int32_t     0 if ok, else return esp_err_t
 */
static int32_t platform_write(I2C_Sensor *handle, uint8_t Reg, const uint8_t *Bufp, uint16_t len)
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

    return handle_err("I2C Platform Write",err);
}

/**
 * @brief    Read generic device register for ESP-IDF based on PID 
 * 
 * @param handle       I2C_Sensor - for setting i2c addr and port number. Can be configured to any other handle for other platforms
 * @param Reg          register to read
 * @param Bufp         pointer to buffer that stores the data read
 * @param len          number of consecutive register to read
 * @return int32_t     0 if okay, else return esp_err_t
 */
static int32_t platform_read(I2C_Sensor *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len)
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

    return handle_err("I2C Platform Write",err);
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

/**
 * @brief Initiate sofware reset of PCF8523. Used as a way to check if the sensor
 * exists on the I2C bus. I2C bus MUST be initialised before calling the function. 
 * 
 * @param i2c_port_num 
 * @return int32_t 
 */
int32_t pcf8523_init(i2c_port_t i2c_port_num)
{
    //Configure I2C Sensor
    pcf8523_config.i2c_sensor_handle.i2c_addr = PCF8523_ADDRESS;
    pcf8523_config.i2c_sensor_handle.i2c_port_num = i2c_port_num;
    pcf_init_called = true;

    // DEPRECEATED INIT -> Used to write write a byte to the address and look for an ack return
    // i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // i2c_master_start(cmd);
    // i2c_master_write_byte(cmd, ( PCF8523_ADDRESS << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
    // i2c_master_stop(cmd);   
    // esp_err_t res = i2c_master_cmd_begin(i2c_port_num, cmd, I2C_MASTER_TIMEOUT_MS/portTICK_PERIOD_MS);
    // i2c_cmd_link_delete(cmd);

    uint8_t buffer[1] = {PCF8523_RST};

    int32_t res = platform_write(
        &(pcf8523_config.i2c_sensor_handle),
        PCF8523_CONTROL_1,
        buffer,
        1);


    if(res == 0) pcf_init_called = true;
    return handle_err("PCF init",res);
    
}
 
int32_t pcf8523_configure_ctrl1(bool capacitance, bool time_mode)
{   
    if(pcf_init_called == false) return PCF8523_INIT_FALSE_ERR;
    int32_t err;

    uint8_t buffer[1];
    err = platform_read(
        &(pcf8523_config.i2c_sensor_handle),
        PCF8523_CONTROL_1,
        buffer,
        1
    );

    uint8_t current_val = buffer[0];

    if(capacitance == PCF8523_CTRL_7PF)     current_val = current_val & 0b01111111;
    else /*PCF8523_CTRL_12PF5*/             current_val = current_val | ~(0b01111111);
    
    if(time_mode == PCF8523_CTRL_12HR_MODE) current_val = current_val | 0b00001000;
    else /*PCF8523_CTRL_24HR_MODE*/         current_val = current_val & ~(0b00001000);

    buffer[0] = current_val;

    err = platform_write(
        &(pcf8523_config.i2c_sensor_handle),
        PCF8523_CONTROL_1,
        buffer,
        1
    );

    return handle_err("Configure ctrl 1",err);

}

/**
 * @brief              Write the current time on the RTC into the Datetime object
 * 
 * @param time_now     Datetime
 * @return int32_t     0 if no errors
 */
int32_t pcf_8523_timenow(DateTime* time_now)
{   
    if(pcf_init_called == false) return PCF8523_INIT_FALSE_ERR;
    uint8_t buffer[7];
    uint8_t res = platform_read(
        &(pcf8523_config.i2c_sensor_handle),
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
 * @return int32_t     0 if no errors
 */
int32_t pcf8523_adjustTime(DateTime* time_now)
{   
    if(pcf_init_called == false) return PCF8523_INIT_FALSE_ERR;
    uint8_t buffer[7] = {bin2bcd(time_now->seconds),
                         bin2bcd(time_now->minutes),
                         bin2bcd(time_now->hours),
                         bin2bcd(time_now->day),
                         bin2bcd(0), // skip weekdays
                         bin2bcd(time_now->month),
                         bin2bcd(time_now->year - 2000U)};
    
    int32_t res = platform_write(
        &(pcf8523_config.i2c_sensor_handle),
        PCF8523_STATUSREG,
        buffer,
        7);

    return res;      
}

void print_datetime(DateTime *datetime)
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




