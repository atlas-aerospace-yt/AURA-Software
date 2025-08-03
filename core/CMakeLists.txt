set(SOURCES ./Src/main.cpp)

idf_component_register(SRCS ${SOURCES}
                        INCLUDE_DIRS . Inc
                        REQUIRES freertos esp_common esp_timer esp_driver_gpio BMP280 ICM20948 INA219 Quat)