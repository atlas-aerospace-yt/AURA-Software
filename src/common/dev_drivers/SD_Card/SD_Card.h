#pragma once

#include "SPI.h"
#include "driver/sdmmc_types.h"
#include "driver/sdspi_host.h"
#include "esp_vfs_fat.h"

namespace sd_card {

constexpr const char *SD_DIR = "/sd_card";
constexpr uint8_t MAX_FILE_COUNT = 5;

//
// A very small class to keep the SD card code neat
//
class sd_card {
 private:
  sdmmc_card_t *_sd_card;

 public:
  //
  // Add the SD card to the SPI bus
  //
  explicit sd_card(gpio_num_t cs_pin, spi_host_device_t spi_bus);

  //
  // Mount the fatfs formatted SD card
  //
  auto mount_sd_card(spi_host_device_t spi_bus) -> void;

  //
  // Unmount the SD card
  //
  auto unmount_sd_card() -> void;
};

}  // namespace sd_card
