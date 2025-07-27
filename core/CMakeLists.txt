set(SOURCES ./Src/main.cpp)

idf_component_register(SRCS ${SOURCES}
                        INCLUDE_DIRS . Inc
                        REQUIRES freertos esp_common Gpio BMP280 ICM20948)