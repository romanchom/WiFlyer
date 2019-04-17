#pragma once

#include <driver/i2c.h>

#include <cstddef>

#include <sstream>

struct I2CBus
{
    explicit I2CBus(gpio_num_t sda, gpio_num_t scl) :
        port(i2c_port_t(0))
    {
        i2c_config_t conf;
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = sda;
        conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
        conf.scl_io_num = scl;
        conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
        conf.master.clk_speed = 400000;

        i2c_param_config(this->port, &conf);
        i2c_driver_install(this->port, conf.mode, 0, 0, 0);
    }

    void transfer(uint8_t const deviceAddress, std::byte const * writeBuffer, size_t const writeSize, std::byte * readBuffer, size_t const readSize)
    {
        auto cmd = i2c_cmd_link_create();

        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, deviceAddress << 1, I2C_MASTER_ACK);
        i2c_master_write(cmd, (uint8_t*) writeBuffer, writeSize, I2C_MASTER_ACK);

        if (readSize != 0) {
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, deviceAddress << 1 | 1, I2C_MASTER_ACK);
            i2c_master_read(cmd, (uint8_t*) readBuffer, readSize, I2C_MASTER_LAST_NACK);
        }
        i2c_master_stop(cmd);

        auto ret = i2c_master_cmd_begin(this->port, cmd, 10);

        i2c_cmd_link_delete(cmd);

        if (ret != ESP_OK) {
            std::stringstream ss;
            ss << "I2C Error: " << int(ret);
            throw std::runtime_error(ss.str());
        }
    }
private:
    i2c_port_t port;
};
