idf_component_register(SRCS ./main_core_1.cpp
                        INCLUDE_DIRS .
                        REQUIRES SPI
                            Flash
                            SD_Card
                            utility
                            fatfs
)
