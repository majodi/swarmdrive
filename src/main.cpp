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
#define _NS_DEF_STEP_FREQ 100
#define _NS_DEF_DIRECTION 1

static void consoleRegistration() {
    sendToConsole(_NS_REG_COMMAND, _NS_C_START, 0, "r", "Run Motor"); // register console commands/parameters
    sendToConsole(_NS_REG_COMMAND, _NS_C_STOP, 0, "s", "Stop Motor");
    sendToConsole(_NS_REG_COMMAND, _NS_C_REVERSE, 0, "rv", "Reverse");
    sendToConsole(_NS_REG_PARAMETER, _NS_P_STEP_FREQ, _NS_DEF_STEP_FREQ, "sf", "Step Freq");
    sendToConsole(_NS_REG_PARAMETER, _NS_P_DIRECTION, _NS_DEF_DIRECTION, "d", "Direction");
    sendToConsole(_NS_REG_PARAMETER, _NS_P_RPM, 0, "--", "RPM");
}

// main task
void mainTask(void *arg) {
    Motor motor((motorConfig *) arg);
    motor.createTimer();
    motor.startTimer(8000);
    printf("SignalRotationAngle: %d\n", motor.getSignalRotationAngle());


    // int lastPotValue = 0;                                       // initialize last pot value
    // int potSteadyCnt = 0;                                       // initialize pot steady counter (0 is undefined >0 is cycles steady (same) value)
//     // mcpwmParams motor = initMotor();                            // initialize motor
//     // startTimer(motor, periodic_timer);                          // start the timer
    while(1) {
        // printf("angle: %d\n", motor.getAngle());
//     //     #ifdef _NS_POT_PIN
//     //     readPot(lastPotValue, potSteadyCnt, motor, periodic_timer);             // get pot value
//     //     #endif
//     //     checkMessages(motor, periodic_timer);                                   // get console messages (commands or parameter updates)
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
    // initConsole();
    // consoleRegistration();
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

//!!! heapsize!!!

    xTaskCreatePinnedToCore(mainTask, "mainTask", 8192, &MC, tskIDLE_PRIORITY, NULL, 1);
    while(1) {
        vTaskDelay(1);
    }
}
