idf_component_register(
    SRCS "log_forwarder.c"
    INCLUDE_DIRS "include"
    REQUIRES mqtt_client esp_timer freertos
    PRIV_REQUIRES mqtt
)
