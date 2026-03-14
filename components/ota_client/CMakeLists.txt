set(project_dir ${CMAKE_CURRENT_SOURCE_DIR}/../../)

idf_component_register(SRCS "ota_client.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES esp_http_client app_update nvs_flash esp_driver_gpio esp_netif esp_wifi protocol_examples_common
                    REQUIRES esp_wifi
                    EMBED_TXTFILES ${project_dir}/server_certs/ca_cert.pem)
