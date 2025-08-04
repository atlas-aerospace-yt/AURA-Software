#include "SPI.h"

void spi::create_spi_bus(void)
{
    spi_bus_config_t spi_config = {};

    spi_config.mosi_io_num = GPIO_NUM_35;
    spi_config.miso_io_num = GPIO_NUM_37;
    spi_config.sclk_io_num = GPIO_NUM_36;

    ESP_ERROR_CHECK(spi_bus_initialize(
        SPI_BUS,
        &spi_config,
        SPI_DMA_CH_AUTO
    ));
}
