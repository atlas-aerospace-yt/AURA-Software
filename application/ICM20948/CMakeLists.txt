set(SOURCES ./ICM20948.cpp)

idf_component_register(SRCS ${SOURCES}
                        INCLUDE_DIRS . 
                        REQUIRES I2C)
