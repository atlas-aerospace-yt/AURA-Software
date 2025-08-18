#include "PCA9685.h"

namespace pca9685 {
//
// Method definitions for the pca9685 class
//
pca9685::pca9685(i2c::i2c_bus* master_bus, uint16_t freq, uint8_t addr)
    : _master_bus(master_bus), _freq(freq), _pca_addr(addr) {
  // Setup the I2C device
  _pca_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  _pca_config.scl_speed_hz = I2C_SPEED;
  _pca_config.device_address = _pca_addr;

  ESP_ERROR_CHECK(_master_bus->add_slave_to_bus(&_pca_config, &_pca_handle));
  // Hobby servos use 50Hz
  _set_freq();
  // Set all channels to be fully off
  ESP_ERROR_CHECK(set_pwm_all(0));
}

auto pca9685::_set_freq() -> void {
  // Calculate the PRE_SCALE value according to datasheet
  const uint8_t pre_scale = static_cast<uint8_t>(PCA_INT_CLK_FREQ / (4096 * _freq) - 1);
  // Put the device in sleep mode
  ESP_ERROR_CHECK(_master_bus->write_byte_i2c(_pca_handle, PCA_MODE1, SLEEP_MODE));
  // Set the pre scale value
  ESP_ERROR_CHECK(_master_bus->write_byte_i2c(_pca_handle, PCA_PRE_SCALE, pre_scale));
  // Wake the device up
  ESP_ERROR_CHECK(_master_bus->write_byte_i2c(_pca_handle, PCA_MODE1, WAKE_MODE));
  // Set the output mode to not be open drain
  ESP_ERROR_CHECK(_master_bus->write_byte_i2c(_pca_handle, PCA_MODE2, MODE2_VAl));
}

[[nodiscard]] auto pca9685::set_pwm(uint8_t ch_num, uint16_t pulse) const -> esp_err_t {
  // Limit the input to suitable values
  const uint8_t ch_num_clamped =
      LED0_ON_L + REG_ADDR_SEP_CNT * std::clamp(ch_num, MIN_CH_NUM, MAX_CH_NUM);
  const uint16_t pulse_off = std::clamp(pulse, MIN_PULSE, MAX_PULSE);
  // Compute the value to write to the registers
  uint8_t pulse_reg_val[REG_ADDR_SEP_CNT] = {0x00, 0x00, static_cast<uint8_t>(pulse_off & 0xFF),
                                             static_cast<uint8_t>((pulse_off >> 8) & 0x1F)};
  // Return the esp_err_t
  return _master_bus->write_bytes_i2c<4>(_pca_handle, ch_num_clamped, pulse_reg_val);
}

[[nodiscard]] auto pca9685::set_pwm_all(uint16_t pulse) const -> esp_err_t {
  // Limit the input to suitable values
  const uint16_t pulse_off = std::clamp(pulse, MIN_PULSE, MAX_PULSE);
  // Compute the value to write to the registers
  uint8_t pulse_reg_val[REG_ADDR_SEP_CNT] = {0x00, 0x00, static_cast<uint8_t>(pulse_off & 0xFF),
                                             static_cast<uint8_t>((pulse_off >> 8) & 0x1F)};
  // Return the esp_err_t
  return _master_bus->write_bytes_i2c<4>(_pca_handle, ALL_LED_ON_L, pulse_reg_val);
}

//
// Method definitions for the pca_esc class
//
pca_esc::pca_esc(pca9685* pca_handle, uint8_t channel)
    : _pca_handle(pca_handle), _channel(channel) {
  set_throttle(0);
}

[[nodiscard]] auto pca_esc::set_throttle(uint8_t percent) const -> esp_err_t {
  const float percent_clamped = static_cast<float>(std::clamp(percent, ESC_OFF, ESC_ON)) / 100.0F;
  const uint16_t pulse = static_cast<uint16_t>(percent_clamped * (_max - _min) + _min);
  return _pca_handle->set_pwm(_channel, pulse);
}

}  // namespace pca9685
