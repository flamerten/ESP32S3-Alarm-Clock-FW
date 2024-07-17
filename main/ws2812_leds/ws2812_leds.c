/**
 * @file ws2812_leds.c
 * @author Samuel Yow
 * 
 */

#include "ws2812_leds.h"

static int32_t handle_esp_err(char *TAG,esp_err_t err)
{
    if(err == ESP_OK){
        return 0;
    }
    else if(err == ESP_ERR_INVALID_ARG){
        ESP_LOGE(TAG,"Parameter Error");
    }
    else if(err == ESP_FAIL){
        ESP_LOGE(TAG,"Failure");
    }
    else if(err == ESP_ERR_INVALID_ARG){
        ESP_LOGE(TAG,"Invalid Argument");
    }
    else{
        ESP_LOGE(TAG,"Unkown Error %d",err);
    }
    return 1;
}

int32_t ws2812_init_leds(led_strip_handle_t *strip_handle, uint32_t led_pin_no, uint32_t num_leds)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = led_pin_no, // The GPIO that connected to the LED strip's data line
        .max_leds = num_leds, // The number of LEDs in the strip,

        //Rest are default
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
        .led_model = LED_MODEL_WS2812, // LED strip model
        .flags.invert_out = false, // whether to invert the output signal (useful when your hardware has a level inverter)
    };

    led_strip_rmt_config_t rmt_config = {
    #if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
        .rmt_channel = 0,
    #else
        .clk_src = RMT_CLK_SRC_DEFAULT, // different clock source can lead to different power consumption
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false, // whether to enable the DMA feature
    #endif
    };

    return handle_esp_err("ws2812_init",led_strip_new_rmt_device(&strip_config, &rmt_config, strip_handle));   
}


int32_t ws2812_clear(led_strip_handle_t strip_handle)
{
    esp_err_t err = led_strip_clear(strip_handle);
    return handle_esp_err("ws2812_clear",err);
}

int32_t ws2812_show(led_strip_handle_t strip_handle)
{
    esp_err_t err = led_strip_refresh(strip_handle);
    return handle_esp_err("ws2812_refresh",err);
}


int32_t ws2812_set_pixel_hsv(led_strip_handle_t strip_handle, uint32_t index, 
    uint16_t hue, uint16_t sat, uint8_t val)
{
    esp_err_t err = led_strip_set_pixel_hsv(strip_handle, index, hue, sat, val );
    return handle_esp_err("ws2812_hsv",err);
}

int32_t ws2812_set_pixel_rgb(led_strip_handle_t strip_handle, uint32_t index,
    uint32_t red, uint32_t green, uint32_t blue)
{
    esp_err_t err = led_strip_set_pixel(strip_handle,index, red, green, blue);
    return handle_esp_err("ws2812_rgb",err);   
}


int32_t ws2812_show_number(uint8_t number, uint8_t digit, led_strip_handle_t strip_handle)
{
    uint8_t starting_index = digit * 7 + (digit > 1);
    uint8_t digit_map = 0;
    uint8_t show_val = 0;

    switch(number){
        case(0): digit_map = DIGIT_MAP_0; break;
        case(1): digit_map = DIGIT_MAP_1; break;
        case(2): digit_map = DIGIT_MAP_2; break;
        case(3): digit_map = DIGIT_MAP_3; break;
        case(4): digit_map = DIGIT_MAP_4; break;
        case(5): digit_map = DIGIT_MAP_5; break;
        case(6): digit_map = DIGIT_MAP_6; break;
        case(7): digit_map = DIGIT_MAP_7; break;
        case(8): digit_map = DIGIT_MAP_8; break;
        case(9): digit_map = DIGIT_MAP_9; break;
    }

    for(int i = 0; i < 7; i++)
    {
        show_val = digit_map & 1;
        if(show_val){
            ws2812_set_pixel_rgb(strip_handle,starting_index,5,5,5);
        }
        else{
            ws2812_set_pixel_rgb(strip_handle,starting_index,0,0,0);
        }

        starting_index = starting_index + 1;
        digit_map = digit_map >> 1;
    }

    return 0;
}
