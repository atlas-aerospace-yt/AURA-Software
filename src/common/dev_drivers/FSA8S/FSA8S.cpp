#include "FSA8S.h"

namespace fsa8s {

receiver::receiver(gpio_num_t rx_pin, uart_port_t uart_port) : _uart_port(uart_port) {
  // Configure UART
  uart_config_t uart_config = {};
  uart_config.baud_rate = BAUD_RATE;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

  // Setup UART by installing drivers and setting pins
  ESP_ERROR_CHECK(uart_param_config(_uart_port, &uart_config));
  ESP_ERROR_CHECK(uart_driver_install(_uart_port, UART_BUF_SIZE, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_set_pin(_uart_port, -1, rx_pin, -1, -1));
}

[[nodiscard]] auto receiver::_get_frame_start() const -> int16_t {
  // Calculate the last possible start point for a complete frame
  const uint8_t last_frame_start_pos = _data_len - 33;
  // Iterate backwards until 0x20 and 0x40 are found
  for (int16_t i = last_frame_start_pos; i >= 0; i--) {
    if (_data[i] == IBUS_START_1 && _data[i + 1] == IBUS_START_2) {
      return i;
    }
  }
  // If no frame start is found
  return -1;
}

auto receiver::_parse_ch_vals(uint8_t frame_start) -> void {
  _ch1 = _data[frame_start + 2] | _data[frame_start + 3] << 8;
  _ch2 = _data[frame_start + 4] | _data[frame_start + 5] << 8;
  _ch3 = _data[frame_start + 6] | _data[frame_start + 7] << 8;
  _ch4 = _data[frame_start + 8] | _data[frame_start + 9] << 8;
  _ch5 = _data[frame_start + 10] | _data[frame_start + 11] << 8;
  _ch6 = _data[frame_start + 12] | _data[frame_start + 13] << 8;
}

auto receiver::update() -> void {
  _data_len = static_cast<int16_t>(uart_read_bytes(_uart_port, &_data, sizeof(_data), TIMEOUT));
  const int16_t indx = _get_frame_start();
  // Check if there is a data frame
  if (indx != -1) {
    _parse_ch_vals(static_cast<uint8_t>(indx));
  }
}

}  // namespace fsa8s
