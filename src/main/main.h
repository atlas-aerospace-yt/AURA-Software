#pragma once

#include <fstream>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_vfs_fat.h"
#include "esp_timer.h"

#include "driver/sdmmc_types.h"
#include "driver/sdspi_host.h"
#include "driver/gpio.h"

#include "BMP280.h"
#include "ICM20948.h"
#include "INA219.h"
#include "Quat.h"

#define TOP ",1,"
#define BOT ",2,"
#define GRAPHING_BEGINNING "l058~"
#define GRAPHING_ENDING "zC43_"
#define GRAPH(name, data, type)         \
    printf(GRAPHING_BEGINNING);         \
    printf(name);                       \
    printf(type);                       \
    printf(data);                       \
    printf(GRAPHING_ENDING);            \

#define HSPI_HOST SPI2_HOST
