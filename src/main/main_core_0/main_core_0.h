#pragma once

#include "BMP280.h"
#include "FSA8S.h"
#include "I2C.h"
#include "ICM20948.h"
#include "INA219.h"
#include "Quat.h"
#include "esp_timer.h"
#include "utility.h"

void control_loop(void);

void core_0_task(void* args);
