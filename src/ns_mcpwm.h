/**************************************************************************/
/*!
    @file     ns_mcpwm.h
    @author   NickStick BV

    @section  HISTORY

    v0.0 - Start of work

    Example code for Space Vector PWM BLDC motor commutation using
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
#include <ams_as5048b.h>

#ifndef _NS_MCPWM_H_
#define _NS_MCPWM_H_

// console commands
#define _NS_C_START 1
#define _NS_C_STOP 2
#define _NS_C_REVERSE 3

// console parameters
#define _NS_P_STEP_FREQ 1
#define _NS_P_DIRECTION 2
#define _NS_P_RPM 3

// defaults
#define _NS_DEF_STEP_FREQ 100
#define _NS_DEF_DIRECTION 1

// pins
#define _NS_TIMER_DEBUG_PIN GPIO_NUM_19
#define _NS_POT_PIN GPIO_NUM_34
#define _NS_RPS_WIRE_SDA 32
#define _NS_RPS_WIRE_SCL 33

// general
#define _NS_POT_FACTOR 8

namespace ns_mcpwm {

    struct mcpwmConfig {
        gpio_num_t pin0A;
        gpio_num_t pin0B;
        gpio_num_t pin1A;
        gpio_num_t pin1B;
        gpio_num_t pin2A;
        gpio_num_t pin2B;
    };
    struct mcpwmParams {
        int svpwm[360] = {50, 52, 53, 55, 56, 58, 59, 61, 62, 64, 65, 67, 68, 70, 71, 73, 74, 75, 77, 78, 80, 81, 83, 84, 85, 87, 88, 89, 91, 92, 93, 94, 94, 95, 95, 95, 96, 96, 96, 97, 97, 97, 98, 98, 98, 98, 99, 99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 99, 99, 99, 99, 99, 98, 98, 98, 98, 97, 97, 97, 96, 96, 96, 95, 95, 95, 94, 94, 93, 94, 94, 95, 95, 95, 96, 96, 96, 97, 97, 97, 98, 98, 98, 98, 99, 99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 99, 99, 99, 99, 99, 98, 98, 98, 98, 97, 97, 97, 96, 96, 96, 95, 95, 95, 94, 94, 93, 92, 91, 89, 88, 87, 85, 84, 83, 81, 80, 78, 77, 75, 74, 73, 71, 70, 68, 67, 65, 64, 62, 61, 59, 58, 56, 55, 53, 52, 50, 49, 47, 45, 44, 43, 41, 40, 38, 37, 35, 34, 32, 31, 29, 28, 26, 25, 24, 22, 21, 19, 18, 16, 15, 14, 12, 11, 9, 8, 7, 7, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 7, 7, 7, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 7, 7, 8, 9, 11, 12, 14, 15, 16, 18, 19, 21, 22, 24, 25, 26, 28, 29, 31, 32, 34, 35, 37, 38, 40, 41, 43, 44, 45, 47, 49};
        int arraySize = sizeof(svpwm)/sizeof(int);
        int phaseShift = arraySize / 3;
        int stepA;
        int stepB;
        int stepC;
        int stepFreq;
        bool running;
        bool lastRunningState;
        bool forward;
        AMS_AS5048B rps;
        int angle;
        int lastAngle;
        int signalRotationAngle;
        int turns;
        int64_t turnTime;
        int turnDuration;
        int64_t consoleUpdate;
    };

    void mcpwmTask(void *arg);

}

#endif // #ifndef _NS_MCPWM_H_
