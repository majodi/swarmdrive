/**************************************************************************/
/*!
    @file     ns_bdc.h
    @author   NickStick BV

    @section  HISTORY

    v0.0 - Start of work

    Motor class for two brushed motors.
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

#include <driver/gpio.h>
#include <driver/mcpwm.h>
#include <soc/mcpwm_struct.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#ifndef _NS_BDC_H_
#define _NS_BDC_H_

struct motorConfig {                                                            // config structure for initializing motor
    gpio_num_t pin0A;                                                           // coil pins A = PWM, B = ENABLE
    gpio_num_t pin0B;
    gpio_num_t pin1A;
    gpio_num_t pin1B;
    gpio_num_t pin2A;
    gpio_num_t pin2B;
    int pwmFreq;                                                                // motor PWM frequency
};

class BMotor {

    public:
        BMotor(motorConfig* MC);                                 // see comments in source
        void setDuty0(int duty);
        void setDuty1(int duty);
        void setDuty2(int duty);
        void setPowerLevel(int level);
        void forward();
        void stop();
        void reverse();
        void right();
        void left();

    private:
        mcpwm_timer_t _hb0 = MCPWM_TIMER_0;                   // timer for half bridge 0
        mcpwm_timer_t _hb1 = MCPWM_TIMER_1;                   // timer for half bridge 1
        mcpwm_timer_t _hb2 = MCPWM_TIMER_2;                   // timer for half bridge 2
        int powerLevel = 50;                                  // power level

        //methods
        void setup_mcpwm_pins(motorConfig* MC);
        void setup_mcpwm_configuration(int pwmFreq);

};

#endif // #ifndef _NS_BDC_H_
