/**
 * @file               lsm6dsox_driver.h
 * @author             Samuel Yow (flamerten@gmail.com)
 * @brief              Function Prototypes for lsm6dsox_driver.c driver file
 */

#ifndef LSM6DSOX_DRV
#define LSM6DSOX_DRV

#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"

#include "esp_log.h"
#include "driver/i2c.h"

#include "lsm6dsox_reg.h"

#define I2C_MASTER_TIMEOUT_MS 1000

/**
 * @brief Struct that holds IMU data. Is passed to platform read and write. Should be declared
 * but not edited directly.
 */
typedef struct LSM_DriverConfig
{   
    i2c_port_t i2c_port_num;
    uint8_t LSM_8bit_addr;

    int16_t Acc_Raw[3];
    float Acc_mg[3];
    lsm6dsox_fs_xl_t Acc_Scale;

    int16_t Gyr_Raw[3];
    float Gyr_mdps[3];
    lsm6dsox_fs_g_t Gyr_Scale;

    stmdev_ctx_t dev_ctx;

}LSM_DriverConfig_t;

/**
 * @brief Assumes only one LSM is connected to the device at once
 * 
 */
//static LSM_DriverConfig lsm6dsox_confifg;

int32_t lsm_init(LSM_DriverConfig_t *sensor_cfg, i2c_port_t i2c_port_num, uint8_t LSM_SA);
int32_t lsm_configure(LSM_DriverConfig_t *sensor_config,
    lsm6dsox_odr_xl_t Acc_Data_Rate, lsm6dsox_fs_xl_t Acc_Scale,
    lsm6dsox_odr_g_t Gyr_Data_Rate,  lsm6dsox_fs_g_t Gyr_Scale);

int32_t lsm_data_ready(LSM_DriverConfig_t *sensor_cfg);

int32_t lsm_update_raw(LSM_DriverConfig_t *sensor_cfg);
int32_t lsm_convert_raw(LSM_DriverConfig_t *sensor_cfg);
int32_t lsm_configure_activity(LSM_DriverConfig_t *sensor_cfg);
int8_t lsm_check_wake(LSM_DriverConfig_t *sensor_cfg);

#endif //LSM6DSOX_DRV