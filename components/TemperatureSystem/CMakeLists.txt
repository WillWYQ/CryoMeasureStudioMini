idf_component_register(
    SRCS
        "TemperatureSystem.c"
        "temperature_adc_port.c"
        "pt1000_model.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_adc
)
