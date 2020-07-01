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

#include <stdint.h>
#include <cmath>
#include <driver/gpio.h>
#include <esp_timer.h>

#include <driver/mcpwm.h>
#include <soc/mcpwm_struct.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "ns_as5048b.h"

#define MP_PI 3.14159265358979
#define RAD(a) ((a)*MP_PI)/180
#define MIN(a,b,c) ((a <= b) && (a <= c) ? a : (b <= a) && (b <= c) ? b : c)
#define MAX(a,b,c) ((a >= b) && (a >= c) ? a : (b >= a) && (b >= c) ? b : c)

#ifndef _NS_SVPWM_H_
#define _NS_SVPWM_H_

// #define _NS_TIMER_DEBUG_PIN GPIO_NUM_33

struct motorConfig {
    gpio_num_t rpsPinSDA;
    gpio_num_t rpsPinSCL;
    gpio_num_t pin0A;
    gpio_num_t pin0B;
    gpio_num_t pin1A;
    gpio_num_t pin1B;
    gpio_num_t pin2A;
    gpio_num_t pin2B;
    int rpsResolution;
    int pwmFreq;
    int stepFreq;
    int torqueAngle;
    int amplitude;
    int moveSteps;
};

class Motor {

    public:
        Motor(motorConfig* MC);
        void stopTimer();
        int getAngle();
        int getSignalRotationAngle();
        int getDirection();
        int getRPM();
        void startMotor();
        void stopMotor();
        void reverseMotor();
        void moveMotor();
        void setStepFreq(int stepFreq);
        void setTorqueAngle(int angle);
        void setAmplitude(int amplitude);
        void setMoveSteps(int steps);

    private:
        //variables
        float _svpwm[360];
        int _arraySize = sizeof(_svpwm) / sizeof(float);
        int _rpsResolution;
        int _phaseShift = _arraySize / 3;
        int _dblPhaseShift = _phaseShift * 2;
        bool _clockwise = true;
        int _signalPosition;
        bool _running = false;
        int _signalRotationAngleR;                              // R means Raw angle (i.e. integer number between 0 - RPS resolution representing 0 - 60 degrees)
        RPS _rps;
        int _angleR = 0;                                        // _Raw_ angle
        int _lastAngleR = 0;                                    // last _Raw_ angle
        int _lastStep = 0;
        esp_timer_handle_t _periodic_timer;
        int _stepFreq = 0;
        uint64_t _timerInterval;
        int _rpm = 0;
        int _torqueAngle = 90;
        int _amplitude = 50;
        int _moveSteps = 0;
        int _moveStepsLeft = 0;
        bool _moveMode = false;

        //methods
        void createTimer();
        void startTimer(uint64_t interval);
        static void onTimer(void *arg);
        void commutate(int step);
        void determineDirection();
        void disengage();
        void determineSignalRotationAngleR();
        void setup_svpwm();
        void setup_mcpwm_pins(motorConfig* MC);
        void setup_mcpwm_configuration(int pwmFreq);

};

#endif // #ifndef _NS_SVPWM_H_
