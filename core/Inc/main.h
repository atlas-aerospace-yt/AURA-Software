#pragma once

// LED Blink Tutorial

/*
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "Gpio.h"

#define milis_ticks(milis) pdMS_TO_TICKS(milis)

class Main final
{
public:
    esp_err_t setup(void);
    void loop(void);

    Gpio::GpioOutput led{GPIO_NUM_14};
};
*/

// I2C Communication Tutorial
/*
#include "stdio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "math.h"

#define milis_ticks(milis) pdMS_TO_TICKS(milis)

#define I2C_MASTER_TIMEOUT_MS 1000

#define BMP_ADDR 0x76
#define BMP_CHIPID 0x58

#define BMP_DIG_T1 0x89
#define BMP_DIG_T2 0x8B
#define BMP_DIG_T3 0x8D

#define BMP_DIG_P1 0x8F
#define BMP_DIG_P2 0x91
#define BMP_DIG_P3 0x93
#define BMP_DIG_P4 0x95
#define BMP_DIG_P5 0x97
#define BMP_DIG_P6 0x99
#define BMP_DIG_P7 0x9B
#define BMP_DIG_P8 0x9D
#define BMP_DIG_P9 0x9F

#define BMP_TEMP_DATA 0xFA
#define BMP_PRESS_DATA 0xF7
*/

#include "stdio.h"
#include "BMP280.h"
