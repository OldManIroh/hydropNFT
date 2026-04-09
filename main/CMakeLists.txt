idf_build_get_property(project_dir PROJECT_DIR)
idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES dht
                    REQUIRES device_control ota_client mqtt_client ads1115 sntp_client log_forwarder espnow_receiver web_server settings esp_wifi mdns esp_netif)
                    
set_target_properties(${COMPONENT_TARGET} PROPERTIES INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}")
