set(SOURCES ./I2C.cpp)

idf_component_register(SRCS ${SOURCES}
                        INCLUDE_DIRS . 
                        REQUIRES esp_driver_i2c)
