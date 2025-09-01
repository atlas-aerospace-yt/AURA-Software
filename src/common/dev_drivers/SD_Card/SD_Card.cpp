#include "SD_Card.h"

namespace sd_card {

sd_card::sd_card(gpio_num_t cs_pin, spi_host_device_t spi_bus) {
  esp_vfs_fat_mount_config_t mount_config = {};
  mount_config.format_if_mount_failed = false;
  mount_config.max_files = MAX_FILE_COUNT;

  sdmmc_host_t sd_host = SDSPI_HOST_DEFAULT();
  sd_host.slot = spi_bus;

  sdspi_device_config_t sd_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  sd_config.gpio_cs = cs_pin;
  sd_config.host_id = spi_bus;

  ESP_ERROR_CHECK(esp_vfs_fat_sdspi_mount(SD_DIR, &sd_host, &sd_config,
                                          &mount_config, &_sd_card));
}

void sd_card::unmount_sd_card() {
  esp_vfs_fat_sdcard_unmount(SD_DIR, _sd_card);
}

}  // namespace sd_card
