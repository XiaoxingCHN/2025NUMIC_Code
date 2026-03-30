idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES Device_WS2812B BSP_Camera Algorithm_AprilTag freertos)
