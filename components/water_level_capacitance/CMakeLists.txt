idf_component_register(SRCS "water_level_capacitance.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES esp_driver_touch_sens touch_sens_examples_common)