#pragma once

#include "driver/gpio.h"
#include "driver/spi_common.h"

namespace spi {
void create_spi_bus(spi_host_device_t spi_bus, gpio_num_t mosi, gpio_num_t miso,
                    gpio_num_t sck);
}
