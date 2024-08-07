#include "porthub.h"

#if defined(ARDUINO)
#include <Wire.h>
#elif defined(ESP_PLATFORM)
#include "driver/i2c.h"
#endif

PortHub::PortHub() {
}

PortHub::PortHub(uint8_t iic_addr, TwoWire *wire_) {
    _iic_addr = iic_addr;
    this->wire = wire_;
}

void PortHub::begin() {
#if defined(ARDUINO)
    this->wire->begin();
#elif defined(ESP_PLATFORM)
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = GPIO_NUM_21; // Change to your SDA pin
    conf.scl_io_num = GPIO_NUM_22; // Change to your SCL pin
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
#endif
}

uint16_t PortHub::hub_a_read_value(uint8_t reg) {
    uint8_t RegValue_L = 0, RegValue_H = 0;
#if defined(ARDUINO)
    this->wire->beginTransmission(_iic_addr);
    this->wire->write(reg | 0x06);
    this->wire->endTransmission();

    this->wire->requestFrom((int)_iic_addr, (int)2);
    while (this->wire->available()) {
        RegValue_L = this->wire->read();
        RegValue_H = this->wire->read();
    }
#elif defined(ESP_PLATFORM)
    uint8_t data[2];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_iic_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg | 0x06, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_iic_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_cmd_link_delete(cmd);
    RegValue_L = data[0];
    RegValue_H = data[1];
#endif
    return (RegValue_H << 8) | RegValue_L;
}

uint8_t PortHub::hub_d_read_value_A(uint8_t reg) {
    uint8_t RegValue = 0;
#if defined(ARDUINO)
    this->wire->beginTransmission(_iic_addr);
    this->wire->write(reg | 0x04);
    this->wire->endTransmission();

    this->wire->requestFrom((int)_iic_addr, (int)1);
    while (this->wire->available()) {
        RegValue = this->wire->read();
    }
#elif defined(ESP_PLATFORM)
    uint8_t data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_iic_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg | 0x04, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_iic_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_cmd_link_delete(cmd);
    RegValue = data;
#endif
    return RegValue;
}

uint8_t PortHub::hub_d_read_value_B(uint8_t reg) {
    uint8_t RegValue = 0;
#if defined(ARDUINO)
    this->wire->beginTransmission(_iic_addr);
    this->wire->write(reg | 0x05);
    this->wire->endTransmission();

    this->wire->requestFrom((int)_iic_addr, (int)1);
    while (this->wire->available()) {
        RegValue = this->wire->read();
    }
#elif defined(ESP_PLATFORM)
    uint8_t data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_iic_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg | 0x05, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_iic_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_cmd_link_delete(cmd);
    RegValue = data;
#endif
    return RegValue;
}

void PortHub::hub_d_wire_value_A(uint8_t reg, uint16_t level) {
#if defined(ARDUINO)
    this->wire->beginTransmission(_iic_addr);
    this->wire->write(reg | 0x00);
    this->wire->write(level & 0xff);
    this->wire->endTransmission();
#elif defined(ESP_PLATFORM)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_iic_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg | 0x00, true);
    i2c_master_write_byte(cmd, level & 0xff, true);
    i2c_master_stop(cmd);
    i2c_cmd_link_delete(cmd);
#endif
}

void PortHub::hub_d_wire_value_B(uint8_t reg, uint16_t level) {
#if defined(ARDUINO)
    this->wire->beginTransmission(_iic_addr);
    this->wire->write(reg | 0x01);
    this->wire->write(level & 0xff);
    this->wire->endTransmission();
#elif defined(ESP_PLATFORM)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_iic_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg | 0x01, true);
    i2c_master_write_byte(cmd, level & 0xff, true);
    i2c_master_stop(cmd);
    i2c_cmd_link_delete(cmd);
#endif
}

void PortHub::hub_a_wire_value_A(uint8_t reg, uint16_t duty) {
#if defined(ARDUINO)
    this->wire->beginTransmission(_iic_addr);
    this->wire->write(reg | 0x02);
    this->wire->write(duty & 0xff);
    this->wire->endTransmission();
#elif defined(ESP_PLATFORM)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_iic_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg | 0x02, true);
    i2c_master_write_byte(cmd, duty & 0xff, true);
    i2c_master_stop(cmd);
    i2c_cmd_link_delete(cmd);
#endif
}

void PortHub::hub_a_wire_value_B(uint8_t reg, uint16_t duty) {
#if defined(ARDUINO)
    this->wire->beginTransmission(_iic_addr);
    this->wire->write(reg | 0x03);
    this->wire->write(duty & 0xff);
    this->wire->endTransmission();
#elif defined(ESP_PLATFORM)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_iic_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg | 0x03, true);
    i2c_master_write_byte(cmd, duty & 0xff, true);
    i2c_master_stop(cmd);
    i2c_cmd_link_delete(cmd);
#endif
}

void PortHub::hub_wire_length(uint8_t reg, uint16_t length) {
#if defined(ARDUINO)
    this->wire->beginTransmission(_iic_addr);
    this->wire->write(reg | 0x08);
    this->wire->write(length & 0xff);
    this->wire->write(length >> 8);
    this->wire->endTransmission();
#elif defined(ESP_PLATFORM)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_iic_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg | 0x08, true);
    i2c_master_write_byte(cmd, length & 0xff, true);
    i2c_master_write_byte(cmd, length >> 8, true);
    i2c_master_stop(cmd);
    i2c_cmd_link_delete(cmd);
#endif
}

void PortHub::hub_wire_index_color(uint8_t reg, uint16_t num, uint8_t r, int8_t g, uint8_t b) {
#if defined(ARDUINO)
    this->wire->beginTransmission(_iic_addr);
    this->wire->write(reg | 0x09);
    this->wire->write(num & 0xff);
    this->wire->write(num >> 8);
    this->wire->write(r);
    this->wire->write(g);
    this->wire->write(b);
    this->wire->endTransmission();
#elif defined(ESP_PLATFORM)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_iic_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg | 0x09, true);
    i2c_master_write_byte(cmd, num & 0xff, true);
    i2c_master_write_byte(cmd, num >> 8, true);
    i2c_master_write_byte(cmd, r, true);
    i2c_master_write_byte(cmd, g, true);
    i2c_master_write_byte(cmd, b, true);
    i2c_master_stop(cmd);
    i2c_cmd_link_delete(cmd);
#endif
}

void PortHub::hub_wire_fill_color(uint8_t reg, uint16_t first, uint16_t count, uint8_t r, int8_t g, uint8_t b) {
#if defined(ARDUINO)
    this->wire->beginTransmission(_iic_addr);
    this->wire->write(reg | 0x0a);
    this->wire->write(first & 0xff);
    this->wire->write(first >> 8);
    this->wire->write(count & 0xff);
    this->wire->write(count >> 8);
    this->wire->write(r);
    this->wire->write(g);
    this->wire->write(b);
    this->wire->endTransmission();
#elif defined(ESP_PLATFORM)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_iic_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg | 0x0a, true);
    i2c_master_write_byte(cmd, first & 0xff, true);
    i2c_master_write_byte(cmd, first >> 8, true);
    i2c_master_write_byte(cmd, count & 0xff, true);
    i2c_master_write_byte(cmd, count >> 8, true);
    i2c_master_write_byte(cmd, r, true);
    i2c_master_write_byte(cmd, g, true);
    i2c_master_write_byte(cmd, b, true);
    i2c_master_stop(cmd);
    i2c_cmd_link_delete(cmd);
#endif
}

void PortHub::hub_wire_setBrightness(uint8_t reg, uint8_t brightness) {
#if defined(ARDUINO)
    this->wire->beginTransmission(_iic_addr);
    this->wire->write(reg | 0x0b);
    this->wire->write(brightness & 0xff);
    this->wire->endTransmission();
#elif defined(ESP_PLATFORM)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_iic_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg | 0x0b, true);
    i2c_master_write_byte(cmd, brightness & 0xff, true);
    i2c_master_stop(cmd);
    i2c_cmd_link_delete(cmd);
#endif
}
