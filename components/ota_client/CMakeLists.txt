set(project_dir ${CMAKE_CURRENT_SOURCE_DIR}/../../)

idf_component_register(SRCS "ota_client.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES esp_http_client app_update nvs_flash esp_netif esp_wifi protocol_examples_common mqtt_client
                    REQUIRES esp_wifi mqtt_client device_control
                    EMBED_TXTFILES ${project_dir}/server_certs/ca_cert.pem)
