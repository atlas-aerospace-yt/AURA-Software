#pragma once

/* Standard */
#include <fstream>

/* FreeRTOS */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/* ESP IDF */
#include "esp_timer.h"

/* Bus drivers*/
#include "SPI.h"
#include "I2C.h"

/* Device drivers */
#include "SD_Card.h"
#include "BMP280.h"
#include "ICM20948.h"
#include "INA219.h"

/* External */
#include "Quat.h"

#define TOP ",1,"
#define BOT ",2,"
#define GRAPHING_BEGINNING "l058~"
#define GRAPHING_ENDING "zC43_"
#define GRAPH(name, data, type)         \
    printf(GRAPHING_BEGINNING);         \
    printf(name);                       \
    printf(type);                       \
    printf("%f", data);                       \
    printf(GRAPHING_ENDING);            \
