idf_component_register(
    SRCS 
        "main.c"
        "pcf8523/pcf8523.c"
        "ws2812_leds/ws2812_leds.c"

    INCLUDE_DIRS
        "."
        "pcf8523"
        "ws2812_leds"
    )
