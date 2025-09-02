#pragma once

#include <cstdint>

#include "driver/gpio.h"

constexpr uint8_t MAX_DELAY = 100;

// Dual core priorities
constexpr uint8_t MOST_PRIORITY = 20;
constexpr uint8_t LEAST_PRIORITY = 5;

// SPI Definitions
constexpr gpio_num_t MISO = GPIO_NUM_37;
constexpr gpio_num_t MOSI = GPIO_NUM_35;
constexpr gpio_num_t SCLK = GPIO_NUM_36;

constexpr gpio_num_t FLASH_CS = GPIO_NUM_21;

constexpr gpio_num_t SD_CARD_CS = GPIO_NUM_13;
constexpr gpio_num_t SD_CARD_DETECT = GPIO_NUM_10;

constexpr gpio_num_t RADIO_RST = GPIO_NUM_11;
constexpr gpio_num_t RADIO_CS = GPIO_NUM_12;

constexpr gpio_num_t AUX_SPI_RST = GPIO_NUM_6;
constexpr gpio_num_t AUX_SPI_CS = GPIO_NUM_7;

// I2C Definitions
constexpr gpio_num_t SDA = GPIO_NUM_8;
constexpr gpio_num_t SCL = GPIO_NUM_9;

// CAN Definitions
constexpr gpio_num_t CAN_RX = GPIO_NUM_47;
constexpr gpio_num_t CAN_TX = GPIO_NUM_48;

// MOSFET Definitions
constexpr gpio_num_t MOSFET_1 = GPIO_NUM_4;
constexpr gpio_num_t MOSFET_2 = GPIO_NUM_5;

// LED Definiton
constexpr gpio_num_t LED = GPIO_NUM_14;

// Radio Defintion
constexpr gpio_num_t RC_RX = GPIO_NUM_1;

// SideKick Function Definitions
constexpr auto TOP = ",1,";
constexpr auto BOT = ",2,";
constexpr auto GRAPHING_BEGINNING = "l058~";
constexpr auto GRAPHING_ENDING = "zC43_";

inline void graph(const char* name, double data, const char* type) {
  printf("%s", GRAPHING_BEGINNING);
  printf("%s", name);
  printf("%s", type);
  printf("%f", data);
  printf("%s", GRAPHING_ENDING);
}

struct sensor_data {
  float gyro_x, gyro_y, gyro_z, dt;
  float temp, height;
  float voltage, current, power;
  float ch1, ch2, ch3, ch4, ch5, ch6;
  float esc0, esc1, esc2, esc3;
  float pid_x, pid_y, pid_z;
  float set_x, set_y, set_z;
  float ori_x, ori_y, ori_z;
};
