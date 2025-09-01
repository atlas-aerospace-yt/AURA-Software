//
// This code is based off of the ESP-IDF example: ext_flash
//
#pragma once

#include "esp_flash.h"
#include "esp_flash_spi_init.h"
#include "esp_partition.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"

namespace flash {

constexpr const char *FLASH_TAG = "flash";
constexpr const char *FLASH_DIR = "/ext_flash";
constexpr const char *PARTITION_LABEL = "ext_flash";

constexpr uint8_t MAX_FILES_OPEN = 5;
constexpr uint8_t FREQ_MHZ = 40;
constexpr uint8_t OFFSET = 0;

//
// A class to handle SPI flash storage (non-volatile)
//
class flash {
 private:
  esp_flash_t *_ext_flash;
  esp_flash_spi_device_config_t _flash_config = {};

  wl_handle_t _s_wl_handle = WL_INVALID_HANDLE;

  const gpio_num_t _flash_cs;

 public:
  //
  // Add a flash chip to the pre-existing SPI bus
  //
  // @param flash_cs the chip select pin of the flash chip
  //
  explicit flash(gpio_num_t flash_cs, spi_host_device_t spi_bus);

  //
  // Forcibly mount a FAT partition (may delete stored data)
  //
  // When using a flash chip for the first time, call this function
  //
  auto force_mount_fat_partition() -> void;

  //
  // Attempt to mount a FAT partition but do not re-try
  //
  auto mount_fat_partition() -> void;

  //
  // Unmount the mounted FAT partition
  //
  auto unmount_fat_partition() -> void;

  //
  // List data partitions on the device
  //
  auto list_data_partitions() -> void;
};

};  // namespace flash
