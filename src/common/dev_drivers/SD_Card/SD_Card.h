#pragma once

#include "driver/sdmmc_types.h"
#include "driver/sdspi_host.h"

#include "esp_vfs_fat.h"

#include "SPI.h"

#define SD_DIR "/sd_card"
#define MAX_FILE_COUNT 5

namespace sd_card
{
    void mount_sd_card(sdmmc_card_t*);
    void unmount_sd_card(sdmmc_card_t*);
}
