/**
 * @file ws2812_leds.h
 * @author Samuel Yow
 * @brief .h file for controlling the ws2812_leds on the ESP32s3 custom PCB using the led_strip esp idf component directory
 * 
 * 
 */
#ifndef WS2812_LEDS
#define WS2812_LEDS

#include <stdint.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "led_strip.h"


#define DIGIT_MAP_0  0b1101111
#define DIGIT_MAP_1  0b0000011
#define DIGIT_MAP_2  0b1110110
#define DIGIT_MAP_3  0b0110111
#define DIGIT_MAP_4  0b0011011
#define DIGIT_MAP_5  0b0111101
#define DIGIT_MAP_6  0b1111101
#define DIGIT_MAP_7  0b0000111
#define DIGIT_MAP_8  0b1111111
#define DIGIT_MAP_9  0b0011111



int32_t ws2812_init_leds(led_strip_handle_t *strip_handle, uint32_t led_pin_no, uint32_t num_leds);
int32_t ws2812_clear(led_strip_handle_t strip_handle);
int32_t ws2812_show(led_strip_handle_t strip_handle);

int32_t ws2812_set_pixel_hsv(led_strip_handle_t strip_handle, uint32_t index, 
    uint16_t hue, uint16_t sat, uint8_t val);
int32_t ws2812_set_pixel_rgb(led_strip_handle_t strip_handle, uint32_t index,
    uint32_t red, uint32_t green, uint32_t blue);

int32_t ws2812_show_number(uint8_t number, uint8_t digit, led_strip_handle_t strip_handle);

#endif //WS2812_LEDS