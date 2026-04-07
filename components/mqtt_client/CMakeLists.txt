idf_component_register(SRCS "mqtt_client.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES mqtt nvs_flash esp_netif app_update freertos sntp_client log_forwarder
                    REQUIRES device_control ads1115
)
