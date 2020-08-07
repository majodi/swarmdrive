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

#include <inttypes.h>

// general macro's and constant defines
#define MP_PI 3.14159265358979                                                  // PI
#define RAD(a) ((a)*MP_PI)/180                                                  // RAD(a) macro
#define MIN(a,b,c) ((a <= b) && (a <= c) ? a : (b <= a) && (b <= c) ? b : c)    // MIN(a,b,c) macro
#define MAX(a,b,c) ((a >= b) && (a >= c) ? a : (b >= a) && (b >= c) ? b : c)    // MAX(a,b,c) macro

// SR = Signal Resolution, the array size for one full signal turn (360 degree turn). One value per degree should be sufficient (360 values)
#define SR 360
// SST = Short Settle Time, uS delay to let motor settle before taking a sensor reading for its position
#define SST 6000

#ifndef _NS_SVPWM_H_
#define _NS_SVPWM_H_

// #define _NS_TIMER_DEBUG_PIN GPIO_NUM_33
// #define _NS_DYNAMIC_DEBUG

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
    int stepFreq;                                                               // step frequency
    int torqueAngle;                                                            // torque angle to use
    int amplitude;                                                              // amplitude percentage
    int moveSteps;                                                              // steps to move on MoveSteps command
};

class Motor {

    public:
        Motor(motorConfig* MC);                                 // see comments in source
        bool isInitialized();
        bool isRunning();
        void startMotor();
        void stopMotor();
        void disengage();
        void recal();
        void test();
        void debugRun();
        void setDirection(bool clockwise);
        void reverseMotor(bool onlyPoles = false);
        void moveMotor(int steps=0);                            // move motor in steps (steps either as parameter for adhoc one-commutate movement or else moveSteps is used)
        int getDirection();
        int getRPM();
        int getAngle();
        int getSignalRotationAngle();
        void setStepFreq(int stepFreq);
        void setTorqueAngle(int angle);
        void setAmplitude(int amplitude);
        void setMoveSteps(int steps);

    private:
        // debug
        bool _debugRun = false;
        int _debug[200];
        int _di = 0;
        // quick lookup values to avoid calculations that are used frequently
        int _phaseShift = SR / 3;                               // 120 degree position (phase shift for 3 coils)
        int _dblPhaseShift = _phaseShift * 2;                   // double phase shift position
        int _quarter = SR / 4;                                  // 90 degree position
        float _SRFactor = SR / 360;                             // factor to calculate angle withing Signal Resolution range
        //variables
        mcpwm_timer_t _coil0 = MCPWM_TIMER_0;                   // timer for coil 0
        mcpwm_timer_t _coil1 = MCPWM_TIMER_1;                   // timer for coil 1
        mcpwm_timer_t _coil2 = MCPWM_TIMER_2;                   // timer for coil 2
        bool _initialized = false;                              // object finished initializing
        float _svpwm[SR];                                       // array for holding SVPWM values
        RPS _rps;                                               // Rptational Position Sensor object
        bool _rpsFrontMount = false;                            // RPS orientation
        int _rpsResolution;                                     // RPS resolution
        bool _clockwise = true;                                 // requested direction
        bool _running = false;                                  // running state of motor
        int _signalRotationAngleR;                              // Angle the motor travels with one full signal cycle, R means Raw angle value
        int _angleR = 0;                                        // _Raw_ angle of motor
        int _lastAngleR = 0;                                    // last _Raw_ angle
        int _lastStep = 0;                                      // last commutate step within signal array
        esp_timer_handle_t _periodic_timer;                     // handle of periodic timer
        int _stepFreq = 0;                                      // step frequency (commutation frequency)
        uint64_t _timerInterval;                                // time interval corresponding with step frequency
        int _delta = 0;                                         // average movement delta angle
        int _noProgressCount = 0;                               // number of times not enough movement
        int _torqueAngle = 90;                                  // torque angle to use
        int _torqueSteps = _torqueAngle * _SRFactor;            // torqueSteps corresponding to torqueAngle (avoid frequent calculation)
        int _amplitude = 50;                                    // amplitude (power) factor
        int _moveSteps = 0;                                     // steps to move on moveSteps command
        int _moveStepsLeft = 0;                                 // steps left for current moveSteps execution (used in closed loop variation)
        bool _moveMode = false;                                 // motor running certain amount of steps due to moveSteps command (used in closed loop variation)

        //methods
        void setup_mcpwm_pins(motorConfig* MC);
        void setup_mcpwm_configuration(int pwmFreq);
        void setup_svpwm();
        void createTimer();
        void startTimer(uint64_t interval);
        void stopTimer();
        static void onTimer(void *arg);
        void home();
        void commutate(int step);
        void determineSignalRotationAngleR();

};

#endif // #ifndef _NS_SVPWM_H_
