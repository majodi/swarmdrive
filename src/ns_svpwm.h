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

// #include <vector>
#include <stdint.h>
#include <driver/gpio.h>
#include <esp_timer.h>

#include <driver/mcpwm.h>
// #include "soc/mcpwm_reg.h"
#include <soc/mcpwm_struct.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "ns_as5048b.h"


//tijdelijk
#include <cmath>
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
};

class Motor {

    public:
        Motor(motorConfig* MC);
        void stopTimer();
        int getAngle();
        int getSignalRotationAngle();
        void startMotor();
        void stopMotor();
        void setStepFreq(uint64_t interval);
        // ...

    private:
        //variables
        float _svpwm[360]; // = {50, 48.4895, 46.9795, 45.4703, 43.9626, 42.4567, 40.9531, 39.4522, 37.9546, 36.4606, 34.9707, 33.4855, 32.0052, 30.5305, 29.0617, 27.5992, 26.1436, 24.6952, 23.2546, 21.8221, 20.3982, 18.9833, 17.5778, 16.1822, 14.7969, 13.4224, 12.059, 10.7071, 9.36724, 8.03973, 6.725, 6.29555, 5.8794, 5.4767, 5.08756, 4.7121, 4.35044, 4.00268, 3.66893, 3.3493, 3.04388, 2.75275, 2.47602, 2.21377, 1.96607, 1.73301, 1.51465, 1.31105, 1.12229, 0.948418, 0.789486, 0.645542, 0.516636, 0.402802, 0.304073, 0.220482, 0.152058, 0.0988159, 0.0607758, 0.0379448, 0.0303345, 0.0379448, 0.0607758, 0.0988159, 0.152058, 0.220482, 0.304073, 0.402802, 0.516636, 0.645542, 0.789486, 0.948418, 1.12229, 1.31105, 1.51465, 1.73301, 1.96607, 2.21377, 2.47602, 2.75275, 3.04388, 3.3493, 3.66893, 4.00268, 4.35044, 4.7121, 5.08756, 5.4767, 5.8794, 6.29555, 6.725, 6.29555, 5.8794, 5.4767, 5.08756, 4.7121, 4.35044, 4.00268, 3.66893, 3.3493, 3.04388, 2.75275, 2.47602, 2.21377, 1.96607, 1.73301, 1.51465, 1.31105, 1.12229, 0.948418, 0.789486, 0.645542, 0.516636, 0.402802, 0.304073, 0.220482, 0.152058, 0.0988159, 0.0607758, 0.0379448, 0.0303345, 0.0379448, 0.0607758, 0.0988159, 0.152058, 0.220482, 0.304073, 0.402802, 0.516636, 0.645542, 0.789486, 0.948418, 1.12229, 1.31105, 1.51465, 1.73301, 1.96607, 2.21377, 2.47602, 2.75275, 3.04388, 3.3493, 3.66893, 4.00268, 4.35044, 4.7121, 5.08756, 5.4767, 5.8794, 6.29555, 6.725, 8.03973, 9.36724, 10.7071, 12.059, 13.4224, 14.7969, 16.1822, 17.5778, 18.9833, 20.3982, 21.8221, 23.2546, 24.6952, 26.1436, 27.5992, 29.0617, 30.5305, 32.0052, 33.4855, 34.9707, 36.4606, 37.9546, 39.4522, 40.9531, 42.4567, 43.9626, 45.4703, 46.9795, 48.4895, 50, 51.5105, 53.0205, 54.5297, 56.0374, 57.5433, 59.0469, 60.5478, 62.0454, 63.5394, 65.0293, 66.5145, 67.9948, 69.4695, 70.9383, 72.4008, 73.8564, 75.3048, 76.7454, 78.1779, 79.6018, 81.0167, 82.4222, 83.8178, 85.2031, 86.5776, 87.941, 89.2929, 90.6328, 91.9603, 93.275, 93.7045, 94.1206, 94.5233, 94.9124, 95.2879, 95.6496, 95.9973, 96.3311, 96.6507, 96.9561, 97.2472, 97.524, 97.7862, 98.0339, 98.267, 98.4854, 98.6889, 98.8777, 99.0516, 99.2105, 99.3545, 99.4834, 99.5972, 99.6959, 99.7795, 99.8479, 99.9012, 99.9392, 99.9621, 99.9697, 99.9621, 99.9392, 99.9012, 99.8479, 99.7795, 99.6959, 99.5972, 99.4834, 99.3545, 99.2105, 99.0516, 98.8777, 98.6889, 98.4854, 98.267, 98.0339, 97.7862, 97.524, 97.2472, 96.9561, 96.6507, 96.3311, 95.9973, 95.6496, 95.2879, 94.9124, 94.5233, 94.1206, 93.7045, 93.275, 93.7045, 94.1206, 94.5233, 94.9124, 95.2879, 95.6496, 95.9973, 96.3311, 96.6507, 96.9561, 97.2472, 97.524, 97.7862, 98.0339, 98.267, 98.4854, 98.6889, 98.8777, 99.0516, 99.2105, 99.3545, 99.4834, 99.5972, 99.6959, 99.7795, 99.8479, 99.9012, 99.9392, 99.9621, 99.9697, 99.9621, 99.9392, 99.9012, 99.8479, 99.7795, 99.6959, 99.5972, 99.4834, 99.3545, 99.2105, 99.0516, 98.8777, 98.6889, 98.4854, 98.267, 98.0339, 97.7862, 97.524, 97.2472, 96.9561, 96.6507, 96.3311, 95.9973, 95.6496, 95.2879, 94.9124, 94.5233, 94.1206, 93.7045, 93.275, 91.9603, 90.6328, 89.2929, 87.941, 86.5776, 85.2031, 83.8178, 82.4222, 81.0167, 79.6018, 78.1779, 76.7454, 75.3048, 73.8564, 72.4008, 70.9383, 69.4695, 67.9948, 66.5145, 65.0293, 63.5394, 62.0454, 60.5478, 59.0469, 57.5433, 56.0374, 54.5297, 53.0205, 51.5105};
        int _arraySize = sizeof(_svpwm) / sizeof(float);
        int _rpsResolution;
        int _phaseShift = _arraySize / 3;
        int _dblPhaseShift = _phaseShift * 2;
        bool _clockwise;
        int _signalPosition;
        bool _running = false;
        int _signalRotationAngleR;
        RPS _rps;
        int _angleR = 0;
        int _lastAngleR = 0;
        int _lastStep = 0;
        esp_timer_handle_t _periodic_timer;                          // timer handle
        uint64_t _stepFreq = 0;

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
