/** 
 *  
 */
#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"

#define I2C_MASTER_TIMEOUT_MS 1000

#define DEFAULT_PRIORITY 0
#define GLITCH_IGNORE_COUNT 7

#define BYTE_MASK 0xff

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
        i2c_bus(gpio_num_t sda, gpio_num_t scl, int priority=DEFAULT_PRIORITY)
            : _sda_pin(sda), _scl_pin(scl), _intr_priority(priority)
        {
            _bus_config.i2c_port = I2C_NUM_0,
            _bus_config.sda_io_num = _sda_pin;
            _bus_config.scl_io_num = _scl_pin;
            _bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
            _bus_config.glitch_ignore_cnt = GLITCH_IGNORE_COUNT;
            _bus_config.intr_priority = _intr_priority;
            _bus_config.flags.enable_internal_pullup = false;

            ESP_ERROR_CHECK(i2c_new_master_bus(&_bus_config, &_bus_handle));
        }
 
        /**
         * Add slave device to the master bus
         * 
         * @param dev_config
         * @param dev_handle
         * @returns esp_err_t
         */
        esp_err_t add_slave_to_bus(
            i2c_device_config_t* dev_config,
            i2c_master_dev_handle_t* dev_handle
        )
        {
            return i2c_master_bus_add_device(_bus_handle,
                dev_config,
                dev_handle
            );
        }

        esp_err_t read_byte_arr_i2c(
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

        esp_err_t master_probe(uint8_t dev_addr);

        /**
         * Read n bytes from a register of an i2c device.
         * 
         * @param dev_handle
         * @param reg_addr
         * @param num_of_bytes : the number of bytes to be read
         * @returns dev_handle
         */
        template<typename T, size_t N>
        T read_bytes_i2c(
             i2c_master_dev_handle_t dev_handle,
             uint8_t reg_addr,
             bool lil_end=false
        )
        {
            T output = 0;
            uint8_t data[N];

            ESP_ERROR_CHECK(read_byte_arr_i2c(
                dev_handle,
                reg_addr,
                data,
                N
            ));

            for (int i=0; i<N; i++)
            {
                if (!lil_end)
                {
                    output |= ((T)data[i]) << ((N - 1 - i) * 8);
                }
                else
                {
                    if (i % 2 == 0)
                    {
                        output |= ((T)data[i + 1]) << ((N - 1 - i) * 8);
                    }
                    else
                    {
                        output |= ((T)data[i - 1]) << ((N - 1 - i) * 8);
                    }
                }
            }   

            return output;
        }

        template<typename T, size_t N>
        void write_bytes_i2c(
            i2c_master_dev_handle_t dev_handle,
            uint8_t reg_addr,
            T data
        )
        {
            uint8_t write_buf[N+1];
            write_buf[0] = reg_addr;

            for (int i=1; i <= N; i++)
            {
                write_buf[i] = static_cast<uint8_t>((data >> ((N-i)*8)) & BYTE_MASK);
            }

            ESP_ERROR_CHECK(i2c_master_transmit(
                dev_handle,
                write_buf,
                sizeof(write_buf),
                pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)
            ));
        }
    };
};
