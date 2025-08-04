idf_component_register(SRCS ./main.cpp
                        INCLUDE_DIRS .
                        REQUIRES esp_timer
                            BMP280 
                            ICM20948
                            INA219
                            Quat
                            SPI
                            SD_Card
)
