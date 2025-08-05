#pragma once

#include "esp_flash.h"
#include "esp_partition.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_flash_spi_init.h"

#include "SPI.h"

#define FLASH_DIR "/ext_flash"
#define MAX_FILES_OPEN 5
#define FREQ_MHZ 40
#define OFFSET 0

#define FLASH_TAG "flash"

namespace flash
{
    class flash
    {
    private:
        esp_flash_t* _ext_flash;
        esp_flash_spi_device_config_t _flash_config = {};

        wl_handle_t _s_wl_handle = WL_INVALID_HANDLE;

        const gpio_num_t _flash_cs;

        const char* _partition_label = "storage";

    public:
        flash(gpio_num_t flash_cs)
            : _flash_cs(flash_cs)
        {
            uint32_t id;

            _flash_config.host_id = SPI_BUS;
            _flash_config.cs_io_num = _flash_cs;
            _flash_config.io_mode = SPI_FLASH_DIO;
            _flash_config.freq_mhz = FREQ_MHZ;

            ESP_ERROR_CHECK(spi_bus_add_flash_device(&_ext_flash, &_flash_config));
            ESP_ERROR_CHECK(esp_flash_init(_ext_flash));
            ESP_ERROR_CHECK(esp_flash_read_id(_ext_flash, &id));

            ESP_LOGI(FLASH_TAG,
                "Initialized external Flash, size=%ld KB, ID=0x%ld",
                _ext_flash->size / 1024,
                id
            );
        }

        void mount_fat_partition(void);
        void unmount_fat_partition(void);
    };
};
