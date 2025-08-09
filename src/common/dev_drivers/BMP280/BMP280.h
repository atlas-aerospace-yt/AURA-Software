//
// BMP280 I2C library.
//
// Datasheet for reference:
// https://www.mouser.co.uk/datasheet/2/783/BST_BMP280_DS001-1509562.pdf
//
#pragma once

#include "I2C.h"

namespace bmp280 {

constexpr uint8_t BMP_ADDR = 0x76;

constexpr uint8_t BMP_DIG_T1 = 0x89;
constexpr uint8_t BMP_DIG_T2 = 0x8B;
constexpr uint8_t BMP_DIG_T3 = 0x8D;

constexpr uint8_t BMP_DIG_P1 = 0x8F;
constexpr uint8_t BMP_DIG_P2 = 0x91;
constexpr uint8_t BMP_DIG_P3 = 0x93;
constexpr uint8_t BMP_DIG_P4 = 0x95;
constexpr uint8_t BMP_DIG_P5 = 0x97;
constexpr uint8_t BMP_DIG_P6 = 0x99;
constexpr uint8_t BMP_DIG_P7 = 0x9B;
constexpr uint8_t BMP_DIG_P8 = 0x9D;
constexpr uint8_t BMP_DIG_P9 = 0x9F;

constexpr uint8_t BMP_ID = 0xD0;         // Chip ID is 0x58 and can be read after pwr on
constexpr uint8_t BMP_RESET = 0xE0;      // Write 0xB6 to reset device fully
constexpr uint8_t BMP_STATUS = 0xF3;     // Bit 3 means measuring, Bit 0 means updated
constexpr uint8_t BMP_CTRL_MEAS = 0xF4;  // [7:5] temp [4:2] pressure [1:0] mode
constexpr uint8_t BMP_CONFIG = 0xF5;     // [7:5] standby [4:2] filter [0] enable SPI

constexpr uint8_t BMP_PRESS_DATA = 0xF7;  // Only 20 bits long i.e. 0xF9 [7:4]
constexpr uint8_t BMP_TEMP_DATA = 0xFA;   // Only 20 bits long i.e. 0xFC [7:4]

constexpr uint8_t TEMP_OVERSAMPLE = 0x01;
constexpr uint8_t PRESS_OVERSAMPLE = 0x01;
constexpr uint8_t NORMAL_MODE = 0x03;

constexpr uint32_t I2C_SPEED = 100000;

constexpr uint32_t TEMP_AND_PRESS_MASK = 0xFFFFF;

class bmp280 {
 private:
  i2c::i2c_bus* _master_bus;

  i2c_master_dev_handle_t _bmp_handle;
  i2c_device_config_t _bmp_config{};

  uint8_t _bmp_addr;
  int32_t _comp_temp{};
  int32_t _raw_temp{};
  int32_t _raw_pressure{};

  uint16_t _dig_t1{};
  int16_t _dig_t2{};
  int16_t _dig_t3{};

  uint16_t _dig_p1{};
  int16_t _dig_p2{};
  int16_t _dig_p3{};
  int16_t _dig_p4{};
  int16_t _dig_p5{};
  int16_t _dig_p6{};
  int16_t _dig_p7{};
  int16_t _dig_p8{};
  int16_t _dig_p9{};

  float _initial_pressure = 0.0F;

  void _set_mode(void);
  void _calibrate(void);
  void _get_compensated_temp(void);

 public:
  explicit bmp280(i2c::i2c_bus* master_bus, uint8_t addr = BMP_ADDR);

  void set_height(float acc_height);

  //
  // Read all registers and calculated the compensated temperature as  it
  // is needed for both pressure and temperature calculations
  //
  void update(void);

  //
  // Calculate the pressure using the equation from the datasheet
  //
  // @returns float the sensed pressure in kPa
  //
  float get_pressure(void) const;

  //
  // Calculate the temperature using the equation from the datasheet
  //
  // @returns float the sensed temperature in degrees Celsius
  //
  float get_temperature(void) const;

  //
  // Calculate the altitude based off of the sensed pressure
  //
  // @returns float the calculated altitude in m
  //
  float get_altitude(void) const;
};
};  // namespace bmp280
