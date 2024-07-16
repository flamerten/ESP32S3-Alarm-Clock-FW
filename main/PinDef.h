/**
 * @file               PinDef.h
 * @author             Samuel Yow (flamerten@gmail.com)
 * @brief              Pin definitions based on custom ESP32S3 PCB 
 */

#define LED0          GPIO_NUM_15
#define LED1          GPIO_NUM_16
 
#define PB1           GPIO_NUM_18
#define PB2           GPIO_NUM_8
 
#define BUCK          GPIO_NUM_9
 
#define BUZZER        GPIO_NUM_7
 
#define NEOPIXEL_PIN  GPIO_NUM_17
#define NEOPIXEL_NUM  30

#define PCB_I2C_SDA   GPIO_NUM_38
#define PCB_I2C_SCL   GPIO_NUM_39
#define I2C_PORT      I2C_NUM_0
 
#define IMU_INT1      GPIO_NUM_40
#define IMU_INT2      GPIO_NUM_41
 
#define RTC_INT1      GPIO_NUM_2
 
#define BATT_ADC      GPIO_NUM_6


