#include "SD_Card.h"

void sd_card::mount_sd_card(sdmmc_card_t *sd_card)
{
    esp_vfs_fat_mount_config_t file_config = {};
    file_config.format_if_mount_failed = false;
    file_config.max_files = MAX_FILE_COUNT;

    sdmmc_host_t sd_host = SDSPI_HOST_DEFAULT();
    sd_host.slot = SPI_BUS;

    sdspi_device_config_t sd_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    sd_config.gpio_cs = GPIO_NUM_13;
    sd_config.host_id = SPI_BUS;

    ESP_ERROR_CHECK(esp_vfs_fat_sdspi_mount(
        SD_DIR,
        &sd_host,
        &sd_config,
        &file_config,
        &sd_card
    ));
}

void sd_card::unmount_sd_card(sdmmc_card_t *sd_card)
{
    esp_vfs_fat_sdcard_unmount(SD_DIR, sd_card);
}
