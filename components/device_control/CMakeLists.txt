idf_component_register(SRCS "device_control.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES freertos
    REQUIRES esp_driver_gpio
)
