//
// An i2c driver to make using the ESP-IDF i2c master easier to use
//
#pragma once

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace i2c {

constexpr uint8_t BYTE_LEN = 8;
constexpr uint16_t I2C_MASTER_TIMEOUT_MS = 1000;
constexpr uint8_t DEFAULT_PRIORITY = 0;
constexpr uint8_t GLITCH_IGNORE_COUNT = 7;
constexpr uint8_t BYTE_MASK = 0xff;

//
// A bus object to provide a convinient way to read, write, and add I2C
// devices
//
class i2c_bus final {
 private:
  i2c_master_bus_handle_t _bus_handle;
  i2c_master_bus_config_t _bus_config = {};

  const gpio_num_t _sda_pin;
  const gpio_num_t _scl_pin;
  const int _intr_priority;

 public:
  i2c_bus(gpio_num_t sda, gpio_num_t scl, int priority = DEFAULT_PRIORITY)
      : _sda_pin(sda), _scl_pin(scl), _intr_priority(priority) {
    _bus_config.i2c_port = I2C_NUM_0, _bus_config.sda_io_num = _sda_pin;
    _bus_config.scl_io_num = _scl_pin;
    _bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    _bus_config.glitch_ignore_cnt = GLITCH_IGNORE_COUNT;
    _bus_config.intr_priority = _intr_priority;
    _bus_config.flags.enable_internal_pullup = false;

    ESP_ERROR_CHECK(i2c_new_master_bus(&_bus_config, &_bus_handle));
  }

  //
  // Add slave device to the master bus
  //
  // @param dev_config the device configuration
  // @param dev_handle the device handle
  //
  // @returns esp_err_t the ESP error code from i2c_master_bus_add_device
  //
  [[nodiscard]] esp_err_t add_slave_to_bus(i2c_device_config_t* dev_config,
                                           i2c_master_dev_handle_t* dev_handle) {
    return i2c_master_bus_add_device(_bus_handle, dev_config, dev_handle);
  }

  //
  // Read an array of bytes from a device on the i2c bus
  //
  // @param dev_handle the device handle
  // @param reg_addr the register address which you start reading from
  // @param data the array the result goes to data (passed in by reference)
  // @param len the number of bytes to be read from the start address
  //
  // @returns esp_err_t the ESP error code from i2c_master_transmit_receive
  //
  [[nodiscard]] static esp_err_t read_byte_arr_i2c(i2c_master_dev_handle_t dev_handle,
                                                   uint8_t reg_addr, uint8_t* data, size_t len) {
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len,
                                       pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
  }

  //
  // Write a byte to a device on the i2c bus
  //
  // @param dev_handle the device handle
  // @param reg_addr the register address which you write to
  // @param data the byte to be written to the memory address
  //
  // @returns esp_err_t the ESP error code from i2c_master_transmit
  //
  [[nodiscard]] static esp_err_t write_byte_i2c(i2c_master_dev_handle_t dev_handle,
                                                uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf),
                               pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
  }

  //
  // Probe a device on the i2c bus
  //
  // @param dev_addr the address of the i2c device to be probed
  //
  // @returns esp_err_t the ESP error code from i2c_master_probe
  //
  [[nodiscard]] esp_err_t master_probe(uint8_t dev_addr) {
    return i2c_master_probe(_bus_handle, dev_addr, -1);
  }

  //
  // Read n bytes from a register of an i2c device.
  //
  // @param dev_handle the device handle
  // @param reg_addr the register address which you start reading from
  // @param num_of_bytes the number of bytes to be read
  //
  // @returns <T> a uint of a suitable length to handle the bytes
  //
  template <typename T, size_t N>
  [[nodiscard]] T read_bytes_i2c(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr,
                                 bool lil_end = false) {
    T output = 0;
    uint8_t data[N];

    ESP_ERROR_CHECK(read_byte_arr_i2c(dev_handle, reg_addr, data, N));

    for (int i = 0; i < N; ++i) {
      T shift = (N - 1 - i) * BYTE_LEN;
      if (N > 1 && lil_end && i % 2 == 0) {
        output |= static_cast<T>(data[i + 1]) << shift;
      } else if (N > 1 && lil_end) {
        output |= static_cast<T>(data[i - 1]) << shift;
      } else {
        output |= static_cast<T>(data[i]) << shift;
      }
    }

    return output;
  }

  //
  // Write n bytes to a register of an i2c device. The data array must have [0] as the register
  // address.
  //
  // @param dev_handle the device handle
  // @param data the bytes to be written to the address
  // @prma len the number, n, of bytes to write
  //
  // @returns esp_err_t the ESP error code from i2c_master_transmit
  //
  template <size_t N>
  [[nodiscard]] esp_err_t write_bytes_i2c(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr,
                                          uint8_t* data) {
    uint8_t buf[N + 1];
    buf[0] = reg_addr;

    for (int i = 1; i <= N; i++) {
      buf[i] = data[i - 1];
    }

    return i2c_master_transmit(dev_handle, buf, N + 1, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
  }
};

}  // namespace i2c
