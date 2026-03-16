idf_component_register(SRCS "sntp_client.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES esp_netif esp_event lwip
                    REQUIRES )
