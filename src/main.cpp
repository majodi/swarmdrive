#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "ns_console.h"
#include "ns_svpwm.h"

using namespace ns_console;

// console commands
#define _NS_C_START 1
#define _NS_C_STOP 2
#define _NS_C_REVERSE 3

// console parameters
#define _NS_P_STEP_FREQ 1
#define _NS_P_DIRECTION 2
#define _NS_P_RPM 3

// defaults
#define _NS_DEF_STEP_FREQ 5000
#define _NS_DEF_DIRECTION 1

void consoleRegistration() {
    sendToConsole(_NS_REG_COMMAND, _NS_C_START, 0, "r", "Run Motor"); // register console commands/parameters
    sendToConsole(_NS_REG_COMMAND, _NS_C_STOP, 0, "s", "Stop Motor");
    sendToConsole(_NS_REG_COMMAND, _NS_C_REVERSE, 0, "rv", "Reverse");
    sendToConsole(_NS_REG_PARAMETER, _NS_P_STEP_FREQ, _NS_DEF_STEP_FREQ, "sf", "Step Freq");
    sendToConsole(_NS_REG_PARAMETER, _NS_P_DIRECTION, _NS_DEF_DIRECTION, "d", "Direction");
    sendToConsole(_NS_REG_PARAMETER, _NS_P_RPM, 0, "--", "RPM");
}

// check if console has command or parameter setting
// void checkMessages(mcpwmParams &motor, esp_timer_handle_t periodic_timer) {
void checkMessages(Motor &motor) {
    consoleMessageStruct consoleMessage = availableConsoleMessage();                    // see if there are new messages from console
    if (consoleMessage.messageType != 0) {                                              // if known message type
        if (consoleMessage.messageType == _NS_COMMAND) {                                // *** if command
            if (consoleMessage.identifier == _NS_C_START) motor.startMotor();           // and command is START then start motor with default step freq
            else if (consoleMessage.identifier == _NS_C_STOP) {                         // if STOP
                motor.stopMotor();                                                      // signal not running
                sendToConsole(_NS_SET_PARAMETER, _NS_P_RPM, 0);                         // ask console to set RPM to zero
            }
//             else if (consoleMessage.identifier == _NS_C_REVERSE) {                      // if REVERSE
//                 motor.forward = !motor.forward;                                         // reverse direction
//                 sendToConsole(_NS_SET_PARAMETER, _NS_P_DIRECTION, motor.forward);       // ask console to let user know (display new state)
//             }
        }
        else if (consoleMessage.messageType == _NS_SET_PARAMETER) {                     // *** if parameter change
//             if (consoleMessage.identifier == _NS_P_DIRECTION) {                         // direction change
//                 if (consoleMessage.value == 0 || consoleMessage.value == 1) {           // if can only be forward or backward
//                     motor.forward = consoleMessage.value;                               // set direction
//                 } else {
//                     consoleLogMessage("dir. error");                                    // else let console log error message
//                     sendToConsole(_NS_SET_PARAMETER, _NS_P_DIRECTION, motor.forward);   // set direction to forward
//                 }
//             }
            if (consoleMessage.identifier == _NS_P_STEP_FREQ) {                         // step frequency change
                motor.setStepFreq(consoleMessage.value);                                // set new step frequency
            }
        }
    }
}

// main task
void mainTask(void *arg) {
    Motor motor((motorConfig *) arg);

    // int lastPotValue = 0;                                       // initialize last pot value
    // int potSteadyCnt = 0;                                       // initialize pot steady counter (0 is undefined >0 is cycles steady (same) value)
//     // mcpwmParams motor = initMotor();                            // initialize motor
//     // startTimer(motor, periodic_timer);                          // start the timer
    while(1) {
        // printf("angle: %d\n", motor.getAngle());
//     //     #ifdef _NS_POT_PIN
//     //     readPot(lastPotValue, potSteadyCnt, motor, periodic_timer);             // get pot value
//     //     #endif
        checkMessages(motor);                                                         // get console messages (commands or parameter updates)
//     //     checkFullTurn(motor);                                                   // check for full turn and update console with RPM and turns
//     //     if (motor.running != motor.lastRunningState) {                          // on running state change
//     //         if (motor.lastRunningState) {                                       // ** were we running?
//     //             ESP_ERROR_CHECK(esp_timer_stop(periodic_timer));                // stop timer (= stop running)
//     //             disengage();                                                    // disengage motor
//     //         } else {                                                            // ** else (we were not running); start timer (start motor)
//     //             ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, (1.0 / motor.stepFreq) * 1000000));
//     //         }
//     //         motor.lastRunningState = motor.running;
//     //     }
        vTaskDelay(1);
    }
}

extern "C" void app_main()
{
    motorConfig MC;
    initConsole();
    consoleRegistration();
    MC.pin0A         = GPIO_NUM_16; //GPIO_NUM_15;
    MC.pin0B         = GPIO_NUM_21; //GPIO_NUM_2;
    MC.pin1A         = GPIO_NUM_17; //GPIO_NUM_0;
    MC.pin1B         = GPIO_NUM_22; //GPIO_NUM_4;
    MC.pin2A         = GPIO_NUM_18; //GPIO_NUM_16;
    MC.pin2B         = GPIO_NUM_23; //GPIO_NUM_17;
    MC.rpsPinSDA     = GPIO_NUM_32;
    MC.rpsPinSCL     = GPIO_NUM_33;
    MC.rpsResolution = 16383; // 2^14 - 1 (is max)
    MC.pwmFreq       = 40000;
    MC.stepFreq      = _NS_DEF_STEP_FREQ;

//!!! heapsize!!!

    xTaskCreatePinnedToCore(mainTask, "mainTask", 8192, &MC, tskIDLE_PRIORITY, NULL, 1);
    while(1) {
        vTaskDelay(1);
    }
}
