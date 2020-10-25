/**************************************************************************/
/*!
    @file     main.cpp (BLDC SVPWM Example)
    @author   NickStick BV

    @section  HISTORY

    v0.0 - Start of work

    Example using Motor class for Space Vector PWM BLDC motor commutation using
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
#include <Arduino.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/adc.h>
#include "ns_console.h"
#include "ns_svpwm.h"

using namespace ns_console;

// console commands
#define _NS_C_START 1
#define _NS_C_STOP 2
#define _NS_C_REVERSE 3
#define _NS_C_MOVE 4
#define _NS_C_DEBUG 6
#define _NS_C_TEST 7
#define _NS_C_RECAL 8

// console parameters
#define _NS_P_STEP_FREQ 1
#define _NS_P_DIRECTION 2
#define _NS_P_RPM 3
#define _NS_P_TORQUE_ANGLE 4
#define _NS_P_AMPLITUDE 5
#define _NS_P_STEPS 6

// defaults
#define _NS_DEF_STEP_FREQ 100
#define _NS_DEF_TORQUE_ANGLE 90
#define _NS_DEF_DIRECTION 1
#define _NS_DEF_AMPLITUDE 50
#define _NS_DEF_STEPS 30

// if you define a POT pin, be sure to connect a potentiometer (else value is floating)
// #define _NS_POT_PIN ADC1_GPIO34_CHANNEL                              // GPIO_NUM_34

// register console commands/parameters
void consoleRegistration() {
    sendToConsole(_NS_REG_COMMAND, _NS_C_RECAL, 0, "rc", "recal");                                  // register recal command
    sendToConsole(_NS_REG_COMMAND, _NS_C_TEST, 0, "t", "test");                                     // register test command
    sendToConsole(_NS_REG_COMMAND, _NS_C_DEBUG, 0, "d", "debugRun");                                // register debugRun command
    sendToConsole(_NS_REG_COMMAND, _NS_C_START, 0, "r", "Run Motor");                               // register run command
    sendToConsole(_NS_REG_COMMAND, _NS_C_STOP, 0, "s", "Stop Motor");                               // stop command
    sendToConsole(_NS_REG_COMMAND, _NS_C_REVERSE, 0, "rv", "Reverse");                              // reverse command
    sendToConsole(_NS_REG_COMMAND, _NS_C_MOVE, 0, "m", "Move Steps");                               // register move steps command (timer mode)
    sendToConsole(_NS_REG_PARAMETER, _NS_P_STEP_FREQ, _NS_DEF_STEP_FREQ, "sf", "Step Freq");        // step frequency parameter
    sendToConsole(_NS_REG_PARAMETER, _NS_P_DIRECTION, _NS_DEF_DIRECTION, "d", "Direction");         // direction parameter
    sendToConsole(_NS_REG_PARAMETER, _NS_P_RPM, 0, "--", "RPM");                                    // rpm parameter
    sendToConsole(_NS_REG_PARAMETER, _NS_P_TORQUE_ANGLE, _NS_DEF_TORQUE_ANGLE, "ta", "Torque A.");  // torque angle parameter
    sendToConsole(_NS_REG_PARAMETER, _NS_P_AMPLITUDE, _NS_DEF_AMPLITUDE, "a", "Amplitude");         // amplitude parameter
    sendToConsole(_NS_REG_PARAMETER, _NS_P_STEPS, _NS_DEF_STEPS, "ms", "Steps");                    // number of steps parameter
}

// check if console has command or parameter setting message available
void checkMessages(Motor &motor) {
    consoleMessageStruct consoleMessage = availableConsoleMessage();                    // see if there are new messages from console
    if (consoleMessage.messageType != 0) {                                              // if known message type
        if (consoleMessage.messageType == _NS_COMMAND) {                                // *** if command
            if (consoleMessage.identifier == _NS_C_START) motor.startMotor();           // and command is START then start motor
            if (consoleMessage.identifier == _NS_C_TEST) motor.test();                  // run test
            if (consoleMessage.identifier == _NS_C_RECAL) motor.recal();                // run recalibrate
            if (consoleMessage.identifier == _NS_C_DEBUG) motor.debugRun();             // turn on debug run
            else if (consoleMessage.identifier == _NS_C_STOP) {                         // if STOP
                motor.stopMotor();                                                      // stop motor
                sendToConsole(_NS_SET_PARAMETER, _NS_P_RPM, 0);                         // ask console to set RPM to zero
            }
            else if (consoleMessage.identifier == _NS_C_REVERSE) {                      // if REVERSE
                motor.reverseMotor();                                                   // reverse direction
                sendToConsole(_NS_SET_PARAMETER, _NS_P_DIRECTION, motor.getDirection()); // ask console to let user know (display new state)
            }
            else if (consoleMessage.identifier == _NS_C_MOVE) motor.moveMotor();        // move motor number of steps (timer mode)
        }
        else if (consoleMessage.messageType == _NS_SET_PARAMETER) {                     // *** if parameter change
            if (consoleMessage.identifier == _NS_P_DIRECTION) {                         // direction change
                if (consoleMessage.value != motor.getDirection()) {                     // and direction changed
                    motor.reverseMotor();                                               // reverse motor
                    sendToConsole(_NS_SET_PARAMETER, _NS_P_DIRECTION, motor.getDirection()); // ask console to let user know (display new state)
                }
            }
            if (consoleMessage.identifier == _NS_P_STEP_FREQ) {                         // step frequency change
                motor.setStepFreq(consoleMessage.value);                                // set new step frequency
            }
            if (consoleMessage.identifier == _NS_P_TORQUE_ANGLE) {                      // step torque angle change
                motor.setTorqueAngle(consoleMessage.value);                             // set new torque angle
            }
            if (consoleMessage.identifier == _NS_P_AMPLITUDE) {                         // step torque angle change
                motor.setAmplitude(consoleMessage.value);                               // set new torque angle
            }
            if (consoleMessage.identifier == _NS_P_STEPS) {                             // change number of steps for move action
                motor.setMoveSteps(consoleMessage.value);                               // set new value for move steps
            }
        }
    }
}

// main task
void mainTask(void *arg) {
    Motor motor((motorConfig *) arg);                                   // create motor instance on this new task thread 
    sendToConsole(_NS_SET_PARAMETER, _NS_P_DIRECTION, motor.getDirection());    // after initialization update console with direction
#ifdef _NS_POT_PIN
    int lastPotVal = 0;                                                 // init pot last known POT value
    int potVal = 0;                                                     // init current raw POT value
    int potMovement = 0;                                                // init last pot movement
    int lastPotFreq = 0;                                                // init last pot frequency value
    int potFreq = 0;                                                    // init current pot frequency value
#endif
    int lastRPM = 0;                                                    // init last known RPM value
    int RPM = 0;                                                        // init current PWM value
    bool engaged = false;                                               // motor coils are in rest at start
    int64_t lastDisengage = 0;                                          // last time disengaged motor coils
    while(1) {                                                          // forever
        // printf("angle: %d\n", motor.getAngle());
        checkMessages(motor);                                           // get console messages (commands or parameter updates) and act upon it
#ifdef _NS_POT_PIN
        if (motor.isInitialized()) potVal = adc1_get_raw(_NS_POT_PIN);  // read raw POT value (but only when motor is initialized else turning can interfere with initialization)
        if (motor.isRunning()) {                                        // if running, use pot for speed control
            potFreq = potVal / 10;                                          // divide for lower more acceptable value for frequency
            if (abs(potFreq - lastPotFreq) > 10) {                          // if pot frequency value changed significantly enough
                motor.setStepFreq(potFreq);                                 // set POT value as frequency (i.e. speed)
                sendToConsole(_NS_SET_PARAMETER, _NS_P_STEP_FREQ, potFreq); // update console to show new frequency
                lastPotFreq = potFreq;
            }
        } else {                                                        // if not running set motor angle with pot
            potMovement = (potMovement + (potVal - lastPotVal)) / 2;    // pot movement to make (averaged to filter out some pot glitches)
            if (abs(potMovement) > 15) {                                // if pot value changed significantly enough
                motor.moveMotor(potMovement);                           // move motor
                engaged = true;                                         // after a move the coils are left engaged, use bool to track this
            }
        }
        lastPotVal = potVal;                                            // remember this last value
#endif
        if (engaged && ((esp_timer_get_time() - lastDisengage) > 500000)) { // if engaged after some time, disengage to avoid overheating
            motor.disengage();                                          // disengage the coils
            engaged = false;                                            // reset engage flag
            lastDisengage = esp_timer_get_time();
        }
        RPM = motor.getRPM();                                           // get current RPM
        if (abs(RPM - lastRPM) > 100) {                                  // if changed enough (filter noise)
            sendToConsole(_NS_SET_PARAMETER, _NS_P_RPM, RPM);           // update console with new RPM value
            lastRPM = RPM;                                              // remember this last value
        }
        vTaskDelay(1);                                                  // keep watch dog happy
    }
}

extern "C" void app_main()
{
    motorConfig MC;                                                     // define motor config
    initConsole(_NS_CON_OPTION_NO_UI);                                  // optional console without GUI for debugging using printf() statements
    // initConsole();                                                      // init console with GUI
    consoleRegistration();                                              // register commands and parameters
    MC.pin0A         = GPIO_NUM_16;                                     // set pins
    MC.pin0B         = GPIO_NUM_21;
    MC.pin1A         = GPIO_NUM_17;
    MC.pin1B         = GPIO_NUM_22;
    MC.pin2A         = GPIO_NUM_18;
    MC.pin2B         = GPIO_NUM_23;
    MC.rpsPinSDA     = GPIO_NUM_32;
    MC.rpsPinSCL     = GPIO_NUM_33;
    MC.rpsResolution = AS5048B_RESOLUTION;                              // resolution of RPS (Rotational Position Sensor) to 14 bit = 2^14 - 1
    MC.rpsFrontMount = false;                                           // senser mounted on the back of the motor
    MC.pwmFreq       = 20000;                                           // pwm frequency above human hearing level
    MC.stepFreq      = _NS_DEF_STEP_FREQ;                               // set default step frequency (also passed to console)
    MC.torqueAngle   = _NS_DEF_TORQUE_ANGLE;                            // set default torque angle
    MC.amplitude     = _NS_DEF_AMPLITUDE;                               // set default amplitude
    MC.moveSteps     = _NS_DEF_STEPS;                                   // set default move steps
#ifdef _NS_POT_PIN 
    adc1_config_width(ADC_WIDTH_BIT_12);                                // prepare ADC for potentiometer controlling step frequency
    adc1_config_channel_atten(_NS_POT_PIN, ADC_ATTEN_DB_11);
#endif
    xTaskCreatePinnedToCore(mainTask, "mainTask", 8192, &MC, tskIDLE_PRIORITY, NULL, 1);    // start main task
    while(1) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
