#pragma once

#include "esp_timer.h"

#include "utility.h"

#include "I2C.h"

#include "BMP280.h"
#include "ICM20948.h"
#include "INA219.h"

#include "Quat.h"

void control_loop(void);

void core_0_task(void *args);
