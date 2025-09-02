#pragma once

#include "BMP280.h"
#include "FSA8S.h"
#include "I2C.h"
#include "ICM20948.h"
#include "INA219.h"
#include "PCA9685.h"
#include "PID.h"
#include "Quat.h"
#include "esp_timer.h"
#include "utility.h"

void control_loop();

void core_0_task(void* args);

auto update_yaw(float curr_yaw, float yaw_rate, float dt) -> float;
