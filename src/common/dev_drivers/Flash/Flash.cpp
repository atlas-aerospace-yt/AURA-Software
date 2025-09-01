#include "Flash.h"

namespace flash {

flash::flash(gpio_num_t flash_cs, spi_host_device_t spi_bus)
    : _flash_cs(flash_cs) {
  uint32_t id;

  _flash_config.host_id = spi_bus;
  _flash_config.cs_io_num = _flash_cs;
  _flash_config.io_mode = SPI_FLASH_DIO;
  _flash_config.freq_mhz = FREQ_MHZ;

  ESP_ERROR_CHECK(spi_bus_add_flash_device(&_ext_flash, &_flash_config));
  ESP_ERROR_CHECK(esp_flash_init(_ext_flash));
  ESP_ERROR_CHECK(esp_flash_read_id(_ext_flash, &id));

  ESP_LOGI(FLASH_TAG, "Initialized external Flash, size=%ld KB, ID=0x%ld",
           _ext_flash->size / 1024, id);
}

auto flash::force_mount_fat_partition() -> void {
  const esp_partition_t *fat_partition;

  ESP_ERROR_CHECK(esp_partition_register_external(
      _ext_flash, OFFSET, _ext_flash->size, PARTITION_LABEL,
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, &fat_partition));

  esp_vfs_fat_mount_config_t mount_config = {};
  mount_config.format_if_mount_failed = true;  // This may erase data
  mount_config.max_files = MAX_FILES_OPEN;
  mount_config.allocation_unit_size = CONFIG_WL_SECTOR_SIZE;
  mount_config.use_one_fat = false;

  ESP_ERROR_CHECK(esp_vfs_fat_spiflash_mount_rw_wl(
      FLASH_DIR, PARTITION_LABEL, &mount_config, &_s_wl_handle));

  ESP_LOGI(FLASH_TAG,
           "Erased flash and mounted FAT partition on external flash.");
}

auto flash::mount_fat_partition() -> void {
  const esp_partition_t *fat_partition;
  ESP_ERROR_CHECK(esp_partition_register_external(
      _ext_flash, OFFSET, _ext_flash->size, PARTITION_LABEL,
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, &fat_partition));

  esp_vfs_fat_mount_config_t mount_config = {};
  mount_config.format_if_mount_failed = false;
  mount_config.max_files = MAX_FILES_OPEN;
  mount_config.allocation_unit_size = CONFIG_WL_SECTOR_SIZE;
  mount_config.use_one_fat = false;

  ESP_ERROR_CHECK(esp_vfs_fat_spiflash_mount_rw_wl(
      FLASH_DIR, PARTITION_LABEL, &mount_config, &_s_wl_handle));

  ESP_LOGI(FLASH_TAG, "Mounted FAT partition on external flash.");
}

auto flash::unmount_fat_partition() -> void {
  ESP_ERROR_CHECK(esp_vfs_fat_spiflash_unmount_rw_wl(FLASH_DIR, _s_wl_handle));

  ESP_LOGI(FLASH_TAG, "Unmounted FAT partition on external flash.");
}

auto flash::list_data_partitions() -> void {
  ESP_LOGI(FLASH_TAG, "Listing data partitions:");
  esp_partition_iterator_t it = esp_partition_find(
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, NULL);

  for (; it != NULL; it = esp_partition_next(it)) {
    const esp_partition_t *part = esp_partition_get(it);
    ESP_LOGI(FLASH_TAG,
             "partition '%s', subtype %d, offset 0x%" PRIx32 ", size %" PRIu32
             " kB",
             part->label, part->subtype, part->address, part->size / 1024);
  }

  esp_partition_iterator_release(it);
}
}  // namespace flash