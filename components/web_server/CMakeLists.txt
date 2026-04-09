idf_component_register(SRCS "web_server.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES esp_http_server esp_wifi esp_netif app_update settings mqtt_client mdns
                    REQUIRES ads1115 device_control sntp_client mqtt_client ota_client
)
