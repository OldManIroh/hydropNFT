idf_component_register(SRCS "mqtt_client.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES mqtt nvs_flash esp_netif protocol_examples_common app_update
                    REQUIRES )
