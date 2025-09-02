idf_component_register(SRCS ./main_core_0.cpp
                        INCLUDE_DIRS .
                        REQUIRES esp_timer
                            utility
                            BMP280 
                            ICM20948
                            INA219
                            Quat
                            FSA8S
                            PCA9685
                            PID
)
