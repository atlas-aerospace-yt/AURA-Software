idf_component_register(SRCS ./Flash.cpp
                        INCLUDE_DIRS .
                        REQUIRES spi_flash
                            esp_partition
                            fatfs
                            vfs 
                            SPI
)
