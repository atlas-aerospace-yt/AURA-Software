#include "Flash.h"

void flash::flash::mount_fat_partition()
{
    const esp_partition_t* fat_partition;
    ESP_ERROR_CHECK(esp_partition_register_external(
        _ext_flash,
        OFFSET,
        _ext_flash->size,
        _partition_label,
        ESP_PARTITION_TYPE_DATA,
        ESP_PARTITION_SUBTYPE_DATA_FAT,
        &fat_partition
    ));

    esp_vfs_fat_mount_config_t mount_config = {};
    mount_config.format_if_mount_failed = false;
    mount_config.max_files = MAX_FILES_OPEN;
    mount_config.allocation_unit_size = CONFIG_WL_SECTOR_SIZE;
    mount_config.use_one_fat = false;

    ESP_ERROR_CHECK(esp_vfs_fat_spiflash_mount_rw_wl(
        FLASH_DIR,
        _partition_label,
        &mount_config,
        &_s_wl_handle
    ));

    ESP_LOGI(FLASH_TAG, "Mounted FAT partition on external flash.");
}

void flash::flash::unmount_fat_partition()
{
    ESP_ERROR_CHECK(esp_vfs_fat_spiflash_unmount_rw_wl(
        FLASH_DIR, _s_wl_handle
    ));

    ESP_LOGI(FLASH_TAG, "Unmounted FAT partition on external flash.");
}
