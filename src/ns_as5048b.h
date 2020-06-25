/**************************************************************************/
/*!
    @file     ns_as5048b.h
    @author   NickStick BV

    @section  HISTORY

    v0.0 - Start of work

    Class for I2C communication with AS5048b Rotation Position Sensor

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2020, NickStick BV
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/
#include <driver/gpio.h>
#include "driver/i2c.h"

#ifndef _NS_AS5048B_H_
#define _NS_AS5048B_H_

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

// AS5048B registers
#define ANGLE_REG 0xFE
#define ZERO_REG 0x16

struct sensorConfig {
    sensorConfig() : i2c_frequency(400000), i2c_port(I2C_NUM_0), chipAddress(0x40) {}
    gpio_num_t i2c_gpio_sda;
    gpio_num_t i2c_gpio_scl;
    int i2c_frequency;
    i2c_port_t i2c_port;
    uint8_t chipAddress;
};

class RPS {

    public:

        RPS();

        void        init(sensorConfig sensorConfig);
        int         resetAngleZero();
        int         getAngleR();
        int         getDataH();
        int         getDataL();

    private:

        uint8_t     _chipAddress;
        i2c_port_t  _i2c_port;
        double      _lastAngle;
        uint8_t     _data_h, _data_l;

        int         readAngle();
        int         writeZeroAngle(uint16_t value);

};

#endif // #ifndef _NS_AS5048B_H_