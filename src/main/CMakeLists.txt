idf_component_register(SRCS ./main.cpp
                        INCLUDE_DIRS .
                        REQUIRES fatfs
                            freertos
                            esp_common
                            esp_timer
                            esp_driver_gpio
                            BMP280 
                            ICM20948
                            INA219
                            Quat
)
