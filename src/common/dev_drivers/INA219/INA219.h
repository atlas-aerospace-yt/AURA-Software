#pragma once

#include "I2C.h"

namespace ina219 {

constexpr uint8_t INA_ADDR = 0x41;

constexpr uint8_t INA_CONFIG = 0x00;
constexpr uint8_t INA_SHUNT_V = 0x01;
constexpr uint8_t INA_BUS_V = 0x02;
constexpr uint8_t INA_POWER = 0x03;
constexpr uint8_t INA_CURRENT = 0x04;
constexpr uint8_t INA_CALIB = 0x05;

constexpr uint32_t I2C_SPEED = 100000;

constexpr uint16_t CALIB_VALUE_1 = 0xD1;
constexpr uint16_t CALIB_VALUE_2 = 0xB8;

constexpr float LSB_BUS_V = 2048.0F;
constexpr float LSB_CURRENT = 327.68F;
constexpr float LSB_POWER = 16.384F;

constexpr float MAX_CURRENT = 150.0F;

//
// A wrapper class to handle communication with an INA219 device connected via the I2c protocol
//
class ina219 {
 private:
  i2c::i2c_bus* _master_bus;

  i2c_master_dev_handle_t _ina_handle;
  i2c_device_config_t _ina_config = {};

  //
  // Write the value to the calibration register which corresponds to a 0m5 ohm shunt resistor
  // and a +- 320mV range
  //
  void _calibrate();

 public:
  explicit ina219(i2c::i2c_bus* master_bus);

  //
  // Read the bus voltage register and convert to V
  //
  // @returns float the bus voltage in V
  //
  [[nodiscard]] auto get_voltage() const -> float;

  //
  // Read the current register and convert to A
  //
  // @returns float the currrent in A
  //
  [[nodiscard]] auto get_current() const -> float;

  //
  // Read the power register and convert to W
  //
  // @returns float the power in W
  //
  [[nodiscard]] auto get_power() const -> float;
};

};  // namespace ina219