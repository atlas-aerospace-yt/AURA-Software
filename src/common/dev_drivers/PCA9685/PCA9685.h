//
// PCA9685 I2C library.
//
// Datasheet for reference:
// https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf
//
#pragma once

#include <algorithm>

#include "I2C.h"

namespace pca9685 {

constexpr uint16_t MIN_PULSE = 0;
constexpr uint16_t MAX_PULSE = 4096;

constexpr uint8_t MIN_CH_NUM = 0;
constexpr uint8_t MAX_CH_NUM = 15;

constexpr uint8_t REG_ADDR_SEP_CNT = 0x04;

constexpr uint32_t I2C_SPEED = 1000000;

constexpr uint8_t ESC_FREQ = 50;
constexpr uint8_t ESC_OFF = 0;
constexpr uint8_t ESC_ON = 100;

constexpr uint32_t PCA_INT_CLK_FREQ = 25000000;

constexpr uint8_t PCA_ADDR = 0x40;

constexpr uint8_t PCA_MODE1 = 0x00;
constexpr uint8_t PCA_MODE2 = 0x01;

constexpr uint8_t LED0_ON_L = 0x06;
constexpr uint8_t LED1_ON_L = 0x10;
constexpr uint8_t LED2_ON_L = 0x14;
constexpr uint8_t LED3_ON_L = 0x18;
constexpr uint8_t LED4_ON_L = 0x22;
constexpr uint8_t LED5_ON_L = 0x26;
constexpr uint8_t LED6_ON_L = 0x30;
constexpr uint8_t LED7_ON_L = 0x34;
constexpr uint8_t LED8_ON_L = 0x38;
constexpr uint8_t LED9_ON_L = 0x42;
constexpr uint8_t LED10_ON_L = 0x46;
constexpr uint8_t LED11_ON_L = 0x50;
constexpr uint8_t LED12_ON_L = 0x54;
constexpr uint8_t LED13_ON_L = 0x58;
constexpr uint8_t LED14_ON_L = 0x62;
constexpr uint8_t LED15_ON_L = 0x66;

constexpr uint8_t ALL_LED_ON_L = 0xFA;
constexpr uint8_t PCA_PRE_SCALE = 0xFE;

// The value for the PRE_SCALE register to set the frequency to 50Hz
constexpr uint8_t SLEEP_MODE = 0x10;
constexpr uint8_t WAKE_MODE = 0x20;
constexpr uint8_t RESTART = 0x80;
constexpr uint8_t MODE2_VAl = 0x04;

//
// A wrapper class to handle communication with a PCA9685 device connected via the I2c protocol
//
class pca9685 {
 private:
  i2c::i2c_bus* _master_bus;

  i2c_master_dev_handle_t _pca_handle;
  i2c_device_config_t _pca_config{};

  const uint16_t _freq;
  const uint8_t _pca_addr;

  //
  // The PRE_SCALE register needs to be set to a frequency based off of the 25MHz internal
  // oscillator
  //
  auto _set_freq() -> void;

 public:
  explicit pca9685(i2c::i2c_bus* master_bus, uint16_t freq = ESC_FREQ, uint8_t addr = PCA_ADDR);

  //
  // Set the pulse width of a specific channel on the PCA9685
  //
  // @param ch_num the channel number (0-15) to edit the pulse width of
  // @param pulse the of the pusle width of the channel (0-4095)/4095
  //
  // @returns esp_err_t the ESP status after writign to the PCA9685
  //
  [[nodiscard]] auto set_pwm(uint8_t ch_num, uint16_t pulse) const -> esp_err_t;

  //
  // Set the pulse width of every channel on the PCA9685 to the same value
  //
  // @param pulse the of the pusle width of all channels (0-4095)/4095
  //
  // @returns esp_err_t the ESP status after writign to the PCA9685
  //
  [[nodiscard]] auto set_pwm_all(uint16_t pulse) const -> esp_err_t;
};

//
// A wrapper class to make using ESCs easier with the PCA9685
//
class pca_esc {
 private:
  pca9685* _pca_handle;

  bool _armed = false;

  const uint8_t _channel;
  const uint16_t _min = 205;
  const uint16_t _max = 310;

 public:
  explicit pca_esc(pca9685* pca_handle, uint8_t channel);

  //
  // Set the throttle percentage of the motor
  //
  // @param percent 0-100, the percent of full throttle to set the motor to
  //
  [[nodiscard]] auto set_throttle(uint8_t percent) const -> esp_err_t;

  //
  // @returns bool the current state of armed
  //
  [[nodiscard]] auto armed() -> bool { return _armed; }

  //
  // Set the ESC to be armed or not armed
  //
  // <<< WARNING >>> When armed, motors may spin - remove propellers
  //
  // @param new_armed the new state of the armed variable
  //
  auto set_armed(bool new_armed) -> void { _armed = new_armed; }
};

}  // namespace pca9685
