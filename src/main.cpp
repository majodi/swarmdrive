test
#include <Arduino.h>
#include "ns_console.h"
#include "ns_mcpwm.h"

// #include <esp_task.h>

// using namespace std;
using namespace ns_console;
using namespace ns_mcpwm;

static void mainSetup() {
    initArduino();
    initConsole();
}

static void taskDispatch() {
    mcpwmConfig mcpwmTaskConfig;
    mcpwmTaskConfig.pin0A = GPIO_NUM_16; //GPIO_NUM_15;
    mcpwmTaskConfig.pin0B = GPIO_NUM_21; //GPIO_NUM_2;
    mcpwmTaskConfig.pin1A = GPIO_NUM_17; //GPIO_NUM_0;
    mcpwmTaskConfig.pin1B = GPIO_NUM_22; //GPIO_NUM_4;
    mcpwmTaskConfig.pin2A = GPIO_NUM_18; //GPIO_NUM_16;
    mcpwmTaskConfig.pin2B = GPIO_NUM_23; //GPIO_NUM_17;
    xTaskCreatePinnedToCore(mcpwmTask, "mcpwmTask", 4096, &mcpwmTaskConfig, tskIDLE_PRIORITY, NULL, 1);
    while(1) {
        vTaskDelay(1);
    }
}

extern "C" void app_main()
{
    mainSetup();
    taskDispatch();
}
