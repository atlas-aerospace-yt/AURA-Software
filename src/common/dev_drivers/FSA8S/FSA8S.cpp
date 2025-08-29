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

auto receiver::_parse_ch_vals(uint8_t frame_start) -> void {
  _ch1 = _data[frame_start + 2] | _data[frame_start + 3] << 8;
  _ch2 = _data[frame_start + 4] | _data[frame_start + 5] << 8;
  _ch3 = _data[frame_start + 6] | _data[frame_start + 7] << 8;
  _ch4 = _data[frame_start + 8] | _data[frame_start + 9] << 8;
  _ch5 = _data[frame_start + 10] | _data[frame_start + 11] << 8;
  _ch6 = _data[frame_start + 12] | _data[frame_start + 13] << 8;
}

auto receiver::update() -> void {
  // Read UART buffer
  _data_len = static_cast<int16_t>(uart_read_bytes(_uart_port, &_data, sizeof(_data), TIMEOUT));

  // Check if there is data to process, then process it
  if (_data_len != 0) {
    _parse_ch_vals(0);
  }
}

}  // namespace fsa8s
