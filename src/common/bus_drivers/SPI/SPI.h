#pragma once

#include "driver/spi_common.h"
#include "driver/gpio.h"

#define SPI_BUS SPI2_HOST

namespace spi
{
    void create_spi_bus(void);
}
