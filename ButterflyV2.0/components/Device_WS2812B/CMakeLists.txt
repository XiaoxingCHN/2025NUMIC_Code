idf_component_register(
    SRCS
        "Device_WS2812B.c"
    INCLUDE_DIRS
        "."
    REQUIRES
    led_strip driver
    )