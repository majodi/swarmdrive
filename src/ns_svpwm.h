/**************************************************************************/
/*!
    @file     ns_svpwm.h
    @author   NickStick BV

    @section  HISTORY

    v0.0 - Start of work

    Motor class for Space Vector PWM BLDC motor commutation using
    the ESP32 mcpwm module. This code uses the ns_console library
    for UI purpose.

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

#include <vector>
#include <stdint.h>
#include <driver/gpio.h>
#include "esp_timer.h"

#ifndef _NS_SVPWM_H_
#define _NS_SVPWM_H_

struct motorConfig {
    motorConfig() : debugPin(GPIO_NUM_NC) {}
    gpio_num_t rpsPinSDA;
    gpio_num_t rpsPinSCL;
    gpio_num_t pin0A;
    gpio_num_t pin0B;
    gpio_num_t pin1A;
    gpio_num_t pin1B;
    gpio_num_t pin2A;
    gpio_num_t pin2B;
    gpio_num_t debugPin;
    int resolution;
    int pwmFreq;
};

class Motor {

    public:
        Motor(motorConfig* MC);
        // ...

    private:
        //variables
        std::vector<float> _svpwm;
        int _resolution;
        int _phaseShift;
        int _stepA;
        int _stepB;
        int _stepC;
        int _stepFreq;
        bool _running;
        bool _lastRunningState;
        bool _forward;
        int _angleStep;
        int _lastAngleStep;
        int _signalRotationAngleStep;
        int _turns;
        int64_t _turnTime;
        int _turnDuration;
        int64_t _consoleUpdate;
        esp_timer_handle_t periodic_timer;                          // timer handle

        //methods
        void setup_mcpwm_pins(motorConfig* MC);
        void setup_mcpwm_configuration(int pwmFreq);

};

#endif // #ifndef _NS_SVPWM_H_
