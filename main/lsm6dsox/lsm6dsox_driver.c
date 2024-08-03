/**
 * @file               lsm6dsox_driver.c
 * @author             Samuel Yow (flamerten@gmail.com)
 * @brief              LSM6DSOX_driver driver file for ESP-iDF, based on ST's lsm6dsox_reg.c 
 *                     PID driver https://github.com/STMicroelectronics/lsm6dsox-pid.
 */

#include "lsm6dsox_driver.h"

//Static Functions

/**
 * @brief    Handle i2c errors in ESP-iDF using esp_log
 * 
 * @param TAG          tag for esp_log
 * @param err          esp_err_t
 * @return int32_t     0 if no error, else return esp_err_t
 */
static int32_t handle_esp_err(char *TAG,esp_err_t err)
{

    if(err == ESP_OK){
        return 0;
    }
    else if(err == ESP_ERR_INVALID_ARG){
        ESP_LOGE(TAG,"Parameter Error");
    }
    else if(err == ESP_ERR_INVALID_ARG){
        ESP_LOGE(TAG,"Sending command error, slave hasn’t ACK the transfer.");
    }
    else if(err == ESP_ERR_INVALID_STATE){
        ESP_LOGE(TAG,"I2C driver not installed or not in master mode");
    }
    else if(err == ESP_ERR_TIMEOUT){
        ESP_LOGE(TAG,"Operation timeout because the bus is busy");
    }
    else{
        ESP_LOGE(TAG,"Unkown Error %d",err);
    }
    return 1;
}

/**
 * @brief    Write generic device register for ESP-IDF based on PID
 * 
 * @param handle       LSM_DriverConfig
 * @param Reg          register to write
 * @param Bufp         pointer to data to write in register reg
 * @param len          number of consecutive register to write
 * @return int32_t     0 if ok, else return esp_err_t
 */
static int32_t platform_write(void *handle, uint8_t Reg, const uint8_t *Bufp, uint16_t len)
{   
    LSM_DriverConfig_t *sensor_config = (LSM_DriverConfig_t *)handle;
    uint8_t sensor_addr = sensor_config->LSM_8bit_addr;
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

    if(err == ESP_OK){
        return 0;
    }
    else{
        return handle_esp_err("platform_write",err);
    }
}

/**
 * @brief    Read generic device register for ESP-IDF based on PID 
 * 
 * @param handle       LSM_DriverConfig 
 * @param Reg          register to read
 * @param Bufp         pointer to buffer that store the data read
 * @param len          number of consecutive register to read
 * @return int32_t     0 if okay, else return esp_err_t
 */
static int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len)
{   
    LSM_DriverConfig_t *sensor_config = (LSM_DriverConfig_t *)handle;
    uint8_t sensor_addr = sensor_config->LSM_8bit_addr;
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

    if(err == ESP_OK){
        return 0;
    }
    else{
        return handle_esp_err("platform_read",err);
    }
}


//Public Functions

/**
 * @brief Initialises the LSM6DSOX unit given the i2c port number and its i2c address. Checks if the sensor exists
 * and then resets the senosr to default config
 * 
 * @param sensor_cfg     
 * @param i2c_port_num   
 * @param LSM_SA         1 if SA is pulled high else 0
 * @return int32_t 
 */
int32_t lsm_init(LSM_DriverConfig_t *sensor_cfg, i2c_port_t i2c_port_num, uint8_t LSM_SA)
{   
    char* TAG = "lsm_init";

    if(LSM_SA) sensor_cfg->LSM_8bit_addr = (LSM6DSOX_I2C_ADD_H >> 1);
    else       sensor_cfg->LSM_8bit_addr = (LSM6DSOX_I2C_ADD_L >> 1); 

    sensor_cfg->i2c_port_num = i2c_port_num;

    sensor_cfg->dev_ctx.write_reg = platform_write;
    sensor_cfg->dev_ctx.read_reg = platform_read;
    sensor_cfg->dev_ctx.handle = sensor_cfg;

    uint8_t whoamI;
    stmdev_ctx_t dev_ctx = sensor_cfg->dev_ctx;
    
    lsm6dsox_device_id_get(&dev_ctx,&whoamI);

    if(whoamI != LSM6DSOX_ID){
        ESP_LOGE(TAG,"Unable to detect sensor is %x should be %x",whoamI,LSM6DSOX_ID);
        while(1);
    }
    else{
        ESP_LOGI(TAG,"Sensor detected");
    }

    //Reset Sensor to default configurations
    uint8_t rst;
    rst = lsm6dsox_reset_set(&dev_ctx,PROPERTY_ENABLE);

    do {
        lsm6dsox_reset_get(&dev_ctx, &rst);
    } while (rst);
    
    return 0;

}

/**
 * @brief Configures the Acc data rate and scale, and the Gyro data rate and scale. Sets block data update
 * and configures the accel to have low pass filter of ODR/100
 * 
 * @param sensor_config 
 * @param Acc_Data_Rate 
 * @param Acc_Scale 
 * @param Gyr_Data_Rate 
 * @param Gyr_Scale 
 * @return int32_t handle_esp_err
 */
int32_t lsm_configure(LSM_DriverConfig_t *sensor_config,
    lsm6dsox_odr_xl_t Acc_Data_Rate, lsm6dsox_fs_xl_t Acc_Scale,
    lsm6dsox_odr_g_t Gyr_Data_Rate,  lsm6dsox_fs_g_t Gyr_Scale)
{
    char* TAG = "lsm_config";

    stmdev_ctx_t dev_ctx = sensor_config->dev_ctx;
    sensor_config->Acc_Scale = Acc_Scale;
    sensor_config->Gyr_Scale = Gyr_Scale;

    int32_t ret = 0;
    ret = lsm6dsox_i3c_disable_set(&dev_ctx, LSM6DSOX_I3C_DISABLE)  & ret;
    ret = lsm6dsox_block_data_update_set(&dev_ctx, PROPERTY_ENABLE) & ret;

    ret = lsm6dsox_xl_data_rate_set(&dev_ctx, Acc_Data_Rate) & ret; 
    ret = lsm6dsox_xl_full_scale_set(&dev_ctx, Acc_Scale)    & ret; 

    ret = lsm6dsox_gy_data_rate_set(&dev_ctx, Gyr_Data_Rate) & ret;
    ret = lsm6dsox_gy_full_scale_set(&dev_ctx, Gyr_Scale)    & ret;

    //Accel Filtering - Low Pass of ODR/100, 
    ret = lsm6dsox_xl_hp_path_on_out_set(&dev_ctx, LSM6DSOX_LP_ODR_DIV_100) & ret;
    ret = lsm6dsox_xl_filter_lp2_set(&dev_ctx, PROPERTY_ENABLE)             & ret;

    //Gyro Filtering - I think not neccessary

    if(ret){
        handle_esp_err(TAG,ret);
    }
    
    return ret;
}

/**
 * @brief    Update sensor_cfg->Acc_Raw and sensor_cfg->Gyr_Raw
 * with values from LSM6DSOX sensor via the polling method
 * 
 * @param sensor_cfg   ptr to LSM_DriverConfig
 * @return int32_t     0 if okay, else return esp_err_t
 */
int32_t lsm_update_raw(LSM_DriverConfig_t *sensor_cfg)
{   
    int32_t err = 0;
    err = lsm6dsox_acceleration_raw_get(&(sensor_cfg->dev_ctx),sensor_cfg->Acc_Raw) & err;
    err = lsm6dsox_angular_rate_raw_get(&(sensor_cfg->dev_ctx),sensor_cfg->Gyr_Raw) & err;

    if(!err){
        return 0;
    }

    handle_esp_err("lsm_raw_fetch",err);
    return err;

}

/**
 * @brief    Based on Gyr and Acc Full scale, convert the raw int16_t values to
 * Gyr(mdps) and Acc(mg) values stored in sensor_cfg->Acc_mg and sensor_cfg->Gyr_mdps 
 * 
 * @param sensor_cfg   ptr to LSM_DriverConfig
 * @return int32_t     returns 0
 */
int32_t lsm_convert_raw(LSM_DriverConfig_t *sensor_cfg)
{
    lsm6dsox_fs_xl_t AccScale = sensor_cfg->Acc_Scale;
    
    float_t (*AccScaleChange)(int16_t);
    switch(AccScale){
        case LSM6DSOX_2g:
            AccScaleChange = &lsm6dsox_from_fs2_to_mg;
            break;
        case LSM6DSOX_16g:
            AccScaleChange = &lsm6dsox_from_fs16_to_mg;
            break;
        case LSM6DSOX_4g:
            AccScaleChange = &lsm6dsox_from_fs4_to_mg;
            break;
        case LSM6DSOX_8g:
            AccScaleChange = &lsm6dsox_from_fs8_to_mg;
            break;
        default:
            AccScaleChange = &lsm6dsox_from_fs8_to_mg;
            ESP_LOGE("LSM_convert","Unspecified Acc Val");
    }

    lsm6dsox_fs_g_t GyrScale = sensor_cfg->Gyr_Scale;
    float_t(*GyrScaleChange)(int16_t);
    switch(GyrScale){
        case LSM6DSOX_250dps:
            GyrScaleChange = &lsm6dsox_from_fs250_to_mdps;
            break;
        case LSM6DSOX_125dps:
            GyrScaleChange = &lsm6dsox_from_fs125_to_mdps;
            break;
        case LSM6DSOX_500dps:
            GyrScaleChange = &lsm6dsox_from_fs500_to_mdps;
            break;
        case LSM6DSOX_1000dps:
            GyrScaleChange = &lsm6dsox_from_fs1000_to_mdps;
            break;
        case LSM6DSOX_2000dps:
            GyrScaleChange = &lsm6dsox_from_fs2000_to_mdps;
            break; 
        default:
            GyrScaleChange = &lsm6dsox_from_fs2000_to_mdps;
            ESP_LOGE("LSM_convert","Unspecified Gyr Val");          
    }

    for(int i = 0; i < 3; i++){
        (sensor_cfg->Acc_mg)[i] = AccScaleChange((sensor_cfg->Acc_Raw)[i]);
        (sensor_cfg->Gyr_mdps)[i] = GyrScaleChange((sensor_cfg->Gyr_Raw)[i]);
    }

    return 0;
}

/**
 * @brief    Given that block data update is set, check that both accel and gyro
 * data is ready to be read.
 * 
 * @param sensor_cfg   ptr to LSM_DriverConfig
 * @return int32_t     0 if okay, else return esp_err_t
 */
int32_t lsm_data_ready(LSM_DriverConfig_t *sensor_cfg)
{
    uint8_t err = 0;
    uint8_t xl_reg,gy_reg;

    err = lsm6dsox_xl_flag_data_ready_get(&(sensor_cfg->dev_ctx),&xl_reg) & err;
    err = lsm6dsox_gy_flag_data_ready_get(&(sensor_cfg->dev_ctx),&gy_reg) & err;

    if(err != 0){
        handle_esp_err("Data_Rdy",err);
        return err;
    }

    return (xl_reg & gy_reg);
}
/**
 * @brief Refer to AN5272: Activity/inactivity and motion/stationary recognition
 * 
 * @param sensor_cfg 
 * @return int32_t 
 */
int32_t lsm_configure_activity(LSM_DriverConfig_t *sensor_cfg)
{
    //Enable interrupt with inactivity configuration
    int err = 0;

    err = err | lsm6dsox_xl_hp_path_internal_set(&(sensor_cfg->dev_ctx), LSM6DSOX_USE_SLOPE); // Or LSM6DSOX_USE_HPF
    err = err | lsm6dsox_wkup_threshold_set(&(sensor_cfg->dev_ctx), 2);

    // /** Set the maximum duration to go into sleep mode/activity mode based on ODR_XL
    //  *  Duration is 4 bits If ODR_XL is 12Hz5,
    //  *  - 512/12.5 = 40.96s of inactivity before sleep occurs.
    //  */
    // err = err | lsm6dsox_act_sleep_dur_set(&(sensor_cfg->dev_ctx),0b0010); //2x40.96 
    // /** Set wakeup duration event. 1 bit 1 ODR time. Number of samples?
    //  *  Duration is 2 bits, If ODR_XL is 12Hz5,
    //  *  Wakeup duration time = 1/12.5 = 0.08s
    //  */
    // err = err | lsm6dsox_wkup_dur_set(&(sensor_cfg->dev_ctx),0b00); //1 sample to wakeup

    // Enable Interrupt
    err = err | lsm6dsox_act_mode_set(&(sensor_cfg->dev_ctx), LSM6DSOX_XL_12Hz5_GY_PD);


    // Drive interrupt to INT1
    lsm6dsox_pin_int1_route_t int1_route_config;
    err = err | lsm6dsox_pin_int1_route_get(&(sensor_cfg->dev_ctx),&int1_route_config);
    int1_route_config.wake_up = PROPERTY_ENABLE;
    err = err | lsm6dsox_pin_int1_route_set(&(sensor_cfg->dev_ctx),int1_route_config);

    return err;
    //return handle_esp_err("lsm configure wake",err);

}

int8_t lsm_check_wake(LSM_DriverConfig_t *sensor_cfg)
{
    lsm6dsox_all_sources_t all_source;
    /* Check if Wake-Up events */
    lsm6dsox_all_sources_get(&(sensor_cfg->dev_ctx), &all_source);

    uint8_t total = 0;

    if (all_source.wake_up) {
      if (all_source.wake_up_x) {
        total += 0b001;
      }

      if (all_source.wake_up_y) {
        total += 0b010;
      }

      if (all_source.wake_up_z) {
        total += 0b100;
      }
    }

    return total;
}

