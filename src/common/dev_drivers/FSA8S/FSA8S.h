#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"

namespace fsa8s {

constexpr uint32_t BAUD_RATE = 115200;

constexpr uint8_t IBUS_START_1 = 0x20;
constexpr uint8_t IBUS_START_2 = 0x40;

constexpr size_t UART_BUF_SIZE = 256;

constexpr TickType_t TIMEOUT = pdMS_TO_TICKS(3);

//
// A receiver object to deal with handling the FlySky iBus protocol
//
class receiver {
 private:
  uart_port_t _uart_port;

  int _data_len{};
  uint8_t _data[UART_BUF_SIZE]{};

  uint16_t _ch1{};
  uint16_t _ch2{};
  uint16_t _ch3{};
  uint16_t _ch4{};
  uint16_t _ch5{};
  uint16_t _ch6{};

  //
  // Convert the data in the buffer into useable values (1000-2000) which are saved in the _ch*
  // variables
  //
  // @params frame_start the index of the start of the full frame
  //
  auto _parse_ch_vals(uint8_t frame_start) -> void;

 public:
  explicit receiver(gpio_num_t rx_pin, uart_port_t uart_port = UART_NUM_1);

  [[nodiscard]] auto ch1() const -> uint16_t { return _ch1; }
  [[nodiscard]] auto ch2() const -> uint16_t { return _ch2; }
  [[nodiscard]] auto ch3() const -> uint16_t { return _ch3; }
  [[nodiscard]] auto ch4() const -> uint16_t { return _ch4; }
  [[nodiscard]] auto ch5() const -> uint16_t { return _ch5; }
  [[nodiscard]] auto ch6() const -> uint16_t { return _ch6; }

  //
  // Read the UART data from the FS-A8S and parse the data to update the channel values
  //
  auto update() -> void;
};

}  // namespace fsa8s
