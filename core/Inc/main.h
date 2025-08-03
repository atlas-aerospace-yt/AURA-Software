#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "stdio.h"
#include "BMP280.h"
#include "ICM20948.h"
#include "INA219.h"
#include "Quat.h"

#define TOP ",1,"
#define BOT ",2,"
#define GRAPHING_BEGINNING "l058~"
#define GRAPHING_ENDING "zC43_"
#define GRAPH(name, data, type) \
    printf(GRAPHING_BEGINNING); \
    printf(name);               \
    printf(type);               \
    printf("%f", data);         \
    printf(GRAPHING_ENDING);       \
