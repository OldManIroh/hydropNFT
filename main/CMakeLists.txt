idf_build_get_property(project_dir PROJECT_DIR)
idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_driver_gpio
                    REQUIRES ota_client mqtt_client ads1115 sntp_client)
                    # water_level_capacitance - временно отключен
