# Вставьте корневой сертификат сервера в окончательный двоичный файл.
idf_build_get_property(project_dir PROJECT_DIR)
idf_component_register(SRCS "native_ota_example.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_http_client app_update nvs_flash esp_driver_gpio esp_netif esp_wifi
                    EMBED_TXTFILES ${project_dir}/server_certs/ca_cert.pem)
