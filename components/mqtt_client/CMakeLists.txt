idf_component_register(SRCS "mqtt_client.c"
                    INCLUDE_DIRS "include" "../../main"
                    PRIV_REQUIRES mqtt nvs_flash esp_netif protocol_examples_common app_update
                    REQUIRES main ads1115
)
