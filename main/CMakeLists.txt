idf_build_get_property(project_dir PROJECT_DIR)
idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_driver_gpio dht
                    REQUIRES ota_client mqtt_client ads1115 sntp_client log_forwarder)
                    
# Экспортируем include директорию для зависимых компонентов
set_target_properties(${COMPONENT_TARGET} PROPERTIES INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}")
