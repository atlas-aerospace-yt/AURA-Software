#pragma once

/* Standard */
#include <fstream>
#include <string>

/* FreeRTOS */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/* Core tasks*/
#include "main_core_0.h"
#include "main_core_1.h"

#define MOST_PRIORITY 20
#define LEAST_PRIORITY tskIDLE_PRIORITY + 1
