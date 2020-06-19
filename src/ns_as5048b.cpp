#include "ns_as5048b.h"
#include "esp32/rom/ets_sys.h"

RPS::RPS() {}

void RPS::init(sensorConfig sensorConfig) {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sensorConfig.i2c_gpio_sda;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = sensorConfig.i2c_gpio_scl;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = sensorConfig.i2c_frequency;
    i2c_param_config(sensorConfig.i2c_port, &conf);
    i2c_driver_install(sensorConfig.i2c_port, I2C_MODE_MASTER, 0, 0, 0);
    _chipAddress = sensorConfig.chipAddress;
    _i2c_port = sensorConfig.i2c_port;
}

int RPS::getDataH() {return _data_h;}

int RPS::getDataL() {return _data_l;}

int RPS::getAngle() {
    int ret = readAngle();
    if (ret == ESP_OK) {
        uint16_t angleVal = _data_h << 6;
        angleVal += (_data_l & 0x3F);
        return angleVal;
    } else {
        return ret > 0 ? -1 * ret : ret;
    }
}

int RPS::resetAngleZero() {
    int ret = writeZeroAngle(0);
    if (ret == ESP_OK) {
        ret = readAngle();
        if (ret == ESP_OK) {
            uint16_t angleVal = _data_h << 6;
            angleVal += (_data_l & 0x3F);
            ret = writeZeroAngle(angleVal);
            if (ret == ESP_OK) {
                return 0;
            } else {
                return ret > 0 ? -1 * ret : ret;
            }
        } else {
            return ret > 0 ? -1 * ret : ret;
        }
    } else {
        return ret > 0 ? -1 * ret : ret;
    }
}

int RPS::readAngle() {
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, _chipAddress << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ANGLE_REG, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(_i2c_port, cmd, 10);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    // ets_delay_us(5);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, _chipAddress << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &_data_l, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &_data_h, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(_i2c_port, cmd, 10);
    i2c_cmd_link_delete(cmd);
    return ret;
}

int RPS::writeZeroAngle(uint16_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, _chipAddress << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ZERO_REG, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (uint8_t) (value >> 6), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (uint8_t) (value & 0x3F), ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(_i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
