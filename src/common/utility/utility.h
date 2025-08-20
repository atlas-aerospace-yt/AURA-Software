#pragma once

#include <cstdint>

#define MAX_DELAY 100

#define TOP ",1,"
#define BOT ",2,"
#define GRAPHING_BEGINNING "l058~"
#define GRAPHING_ENDING "zC43_"
#define GRAPH(name, data, type) \
  printf(GRAPHING_BEGINNING);   \
  printf(name);                 \
  printf(type);                 \
  printf("%f", data);           \
  printf(GRAPHING_ENDING);

struct sensor_data {
  float gyro_x, gyro_y, gyro_z, dt;
  float temp, height;
  float voltage, current, power;
  uint16_t ch1, ch2, ch3, ch4, ch5, ch6;
};
