set(SOURCES ./I2C.h)

idf_component_register(SRCS ${SOURCES}
                        INCLUDE_DIRS . 
                        REQUIRES esp_driver_i2c)
