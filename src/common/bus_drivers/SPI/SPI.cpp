#include "SPI.h"

void spi::create_spi_bus(spi_host_device_t spi_bus, gpio_num_t mosi,
                         gpio_num_t miso, gpio_num_t sck) {
  spi_bus_config_t spi_config = {};

  spi_config.mosi_io_num = mosi;
  spi_config.miso_io_num = miso;
  spi_config.sclk_io_num = sck;

  ESP_ERROR_CHECK(spi_bus_initialize(spi_bus, &spi_config, SPI_DMA_CH_AUTO));
}
