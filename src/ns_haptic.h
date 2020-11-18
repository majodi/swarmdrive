/**************************************************************************/
/*!
    @file     ns_haptic.h
    @author   NickStick BV

    @section  HISTORY

    v0.0 - Start of work

    Motor class for haptic feedback with BLDC motors (Haptic Rotary Encoder).
    This code uses the ns_console library for UI purpose.

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

#include <cmath>
#include <esp_timer.h>
#include <driver/gpio.h>
#include <driver/mcpwm.h>
#include <soc/mcpwm_struct.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "ns_as5048b.h"

// general macro's and constant defines
#define MP_PI 3.14159265358979                                                  // PI
#define RAD(a) ((a)*MP_PI)/180                                                  // RAD(a) macro
#define MIN(a,b,c) ((a <= b) && (a <= c) ? a : (b <= a) && (b <= c) ? b : c)    // MIN(a,b,c) macro
#define MAX(a,b,c) ((a >= b) && (a >= c) ? a : (b >= a) && (b >= c) ? b : c)    // MAX(a,b,c) macro

// SR = Signal Resolution, the array size for one full signal turn (360 degree turn). One value per degree should be sufficient (360 values)
#define SR 360
// SST = Short Settle Time, uS delay to let motor settle before taking a sensor reading for its position
#define SST 6000

#ifndef _NS_HAPTIC_H_
#define _NS_HAPTIC_H_

struct motorConfig {                                                            // config structure for initializing motor
    gpio_num_t rpsPinSDA;                                                       // RPS sensor data pin
    gpio_num_t rpsPinSCL;                                                       // RPS sensor clock pin
    gpio_num_t pin0A;                                                           // coil pins A = PWM, B = ENABLE
    gpio_num_t pin0B;
    gpio_num_t pin1A;
    gpio_num_t pin1B;
    gpio_num_t pin2A;
    gpio_num_t pin2B;
    int rpsResolution;                                                          // resolution of RPS
    bool rpsFrontMount;                                                         // orientation of RPS
    int pwmFreq;                                                                // motor PWM frequency
    int sampleFreq;                                                             // sample frequency
    int powerLevel;                                                             // power level
    int repeatAngleDeg;                                                         // repeat angle in degrees
};

class HRE {

    public:
        HRE(motorConfig* MC);                                 // Haptic Rotary Encoder, see comments in source
        // void reverseMotor(bool onlyPoles = false);
        // void disengage();
        int getAngle();
        int getSpeed();
        int getDirection();
        void setSampleFreq(int sampleFreq);
        void setPowerLevel(int power);
        void setRepeatAngle(int degAngle);

        void handle(int angle);

    private:
        mcpwm_timer_t _coil0 = MCPWM_TIMER_0;                   // timer for half bridge 0
        mcpwm_timer_t _coil1 = MCPWM_TIMER_1;                   // timer for half bridge 1
        mcpwm_timer_t _coil2 = MCPWM_TIMER_2;                   // timer for half bridge 2
        // quick lookup values to avoid calculations that are used frequently
        int _phaseShift = SR / 3;                               // 120 degree position (phase shift for 3 coils)
        int _dblPhaseShift = _phaseShift * 2;                   // double phase shift position
        int _quarter = SR / 4;                                  // 90 degree position
        float _SRFactor = SR / 360;                             // factor to calculate angle withing Signal Resolution range
        int _powerLevel = 20;                                   // power level
        float _sine[SR];                                        // array for holding SVPWM values
        RPS _rps;                                               // Rptational Position Sensor object
        bool _rpsFrontMount = false;                            // RPS orientation
        int _rpsResolution;                                     // RPS resolution
        bool _clockwise = true;                                 // requested direction
        int _dir = 1;                                           // direction 1=CW -1=CCW
        int _signalRotationAngleR;                              // Angle the motor travels with one full signal cycle, R means Raw angle value
        int _angleR = 0;                                        // _Raw_ angle of motor
        int _lastAngleR = 0;                                    // last _Raw_ angle
        int _currentDelta = 0;                                  // current movement delta
        int _avgDelta = 0;                                      // average over last 100 samples
        int _lastDelta = 0;                                     // last movement delta
        int _lastStep = 0;                                      // last commutate step within signal array
        esp_timer_handle_t _periodic_timer;                     // handle of periodic timer
        int _sampleFreq;                                        // sample frequency
        int _sampleCnt = 0;                                     // sample counter
        int _multiSampleStartAngle = 0;                         // start sampling angle
        // bool _inRegion = false;                                 // in region of effect
        // TickType_t _enteredRegion = 0;                          // time entered effect region
        // int _enterDir = 0;                                      // entered from what side
        int _repeatAngleR = 30;                                 // repeat angle raw

        //methods
        void setup_mcpwm_pins(motorConfig* MC);
        void setup_mcpwm_configuration(int pwmFreq);
        void setup_sinepwm();
        void createTimer();
        void startTimer(uint64_t interval);
        void stopTimer();
        static void onTimer(void *arg);
        void home();
        void commutate(int step);
        // void determineSignalRotationAngleR();

};

#endif // #ifndef _NS_HAPTIC_H_
