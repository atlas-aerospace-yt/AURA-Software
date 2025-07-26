/** 
 *  
 */
#pragma once

#include "stdio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"

#define I2C_MASTER_TIMEOUT_MS 1000

#define GLITCH_IGNORE_COUNT 7
#define TRANS_QUEUE_DEPTH 8

namespace i2c
{
    /** 
     *  A class to hold wrapper functions for I2C communication
     */
    class i2c_bus
    {
    private:
        i2c_master_bus_handle_t _bus_handle;
        i2c_master_bus_config_t _bus_config = {};

        const gpio_num_t _sda_pin;
        const gpio_num_t _scl_pin;
        const int _intr_priority;
    public:
        i2c_bus(gpio_num_t sda, gpio_num_t scl, int priority = 0)
            : _sda_pin(sda), _scl_pin(scl), _intr_priority(priority)
        {
            _bus_config.i2c_port = I2C_NUM_0,
            _bus_config.sda_io_num = _sda_pin;
            _bus_config.scl_io_num = _scl_pin;
            _bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
            _bus_config.glitch_ignore_cnt = GLITCH_IGNORE_COUNT;
            _bus_config.intr_priority = _intr_priority;
            _bus_config.trans_queue_depth = TRANS_QUEUE_DEPTH;
            _bus_config.flags.enable_internal_pullup = false;

            ESP_ERROR_CHECK(i2c_new_master_bus(&_bus_config, &_bus_handle));
        }

        esp_err_t read_byte_i2c(
            i2c_master_dev_handle_t dev_handle,
            uint8_t reg_addr,
            uint8_t *data,
            size_t len
        );

        esp_err_t write_byte_i2c(
            i2c_master_dev_handle_t dev_handle,
            uint8_t reg_addr,
            uint8_t data
        );
    };
};
