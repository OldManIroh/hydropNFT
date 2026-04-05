idf_component_register(SRCS "espnow_receiver.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi mqtt_client esp_hw_support
)
