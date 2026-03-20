idf_component_register(
    SRCS
        "display_system.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_driver_i2c
        esp_lcd
        esp_hw_support
)
