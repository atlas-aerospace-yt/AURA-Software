idf_component_register(SRCS ./SD_Card.cpp
                        INCLUDE_DIRS .
                        REQUIRES fatfs
                            SPI
)
