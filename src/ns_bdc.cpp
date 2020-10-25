#include "ns_bdc.h"

/*========================================================================*/
/*                            CONSTRUCTOR(s)                              */
/*========================================================================*/

BMotor::BMotor(motorConfig* MC) {
    setup_mcpwm_pins(MC);                                               // configure mcpwm pins
    setup_mcpwm_configuration(MC->pwmFreq);                             // configure mcpwm with pwm frequency (high enough to be not audible)
}

/*========================================================================*/
/*                            PUBLIC METHODS                              */
/*========================================================================*/


/*========================================================================*/
/*                            PRIVATE METHODS                             */
/*========================================================================*/

// setup pins
void BMotor::setup_mcpwm_pins(motorConfig* MC) {
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MC->pin0A);                  // half bridge 0 high side
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MC->pin0B);                  // driver enable signal to enable half bridge 0 low side
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, MC->pin1A);                  // half bridge high side 1
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, MC->pin1B);                  // driver enable signal to enable half bridge 1 low side
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, MC->pin2A);                  // half bridge high side 2
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, MC->pin2B);                  // driver enable signal to enable half bridge 2 low side
}

// setup mcpwm configuration
void BMotor::setup_mcpwm_configuration(int pwmFreq) {
    mcpwm_config_t pwm_config;                                          // mcpwm config parameter
    pwm_config.frequency = pwmFreq * 2;                                 // up-down counter uses two half cycles so for full cycle to have desired freq it has to be doubled
    pwm_config.cmpr_a = 50.0;                                           // duty cycle of PWMxA = 50.0% (temporary)
    pwm_config.cmpr_b = 50.0;                                           // duty cycle of PWMxB = 50.0% (temporary)
    pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER;                    // Up-down counter (triangle wave)
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;                           // Active HIGH
    mcpwm_init(MCPWM_UNIT_0, _hb0, &pwm_config);                        // Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_0, _hb1, &pwm_config);                        // Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_0, _hb2, &pwm_config);                        // Configure PWM0A & PWM0B with above settings
    mcpwm_sync_enable(MCPWM_UNIT_0, _hb0, MCPWM_SELECT_TIMER0, 0);      // !!! MCPWM_SELECT_TIMER didn't exist in mcpwm.h, had to edit the header
    mcpwm_sync_enable(MCPWM_UNIT_0, _hb1, MCPWM_SELECT_TIMER0, 0);      // it sets to sync all timers to timer0
    mcpwm_sync_enable(MCPWM_UNIT_0, _hb2, MCPWM_SELECT_TIMER0, 0);      // see Github issue https://github.com/espressif/esp-idf/issues/5429
    MCPWM0.timer[0].sync.out_sel = 1;                                   // timer 0 as sync input (https://github.com/espressif/esp-idf/issues/3567)
    vTaskDelay(500/portTICK_PERIOD_MS);                                 // wait long enough for large period PWM (low frequency) to be able to sysnc
    MCPWM0.timer[0].sync.out_sel = 0;                                   // done syncing
    mcpwm_set_duty(MCPWM_UNIT_0, _hb0, MCPWM_OPR_A, 0);                 // set initial duty 0 (disengage motor)
    mcpwm_set_duty(MCPWM_UNIT_0, _hb0, MCPWM_OPR_B, 100);               // set enable (low side of half bridge) to continues High, will switch low due to inverter in L6234 when high side is high
    mcpwm_set_duty(MCPWM_UNIT_0, _hb1, MCPWM_OPR_A, 0);                 // same for hb
    mcpwm_set_duty(MCPWM_UNIT_0, _hb1, MCPWM_OPR_B, 100);               // same for hb
    mcpwm_set_duty(MCPWM_UNIT_0, _hb2, MCPWM_OPR_A, 0);                 // same for hb
    mcpwm_set_duty(MCPWM_UNIT_0, _hb2, MCPWM_OPR_B, 100);               // same for hb
}

void BMotor::setDuty0(int duty) {                                       // set half bridge duty cycle
    mcpwm_set_duty(MCPWM_UNIT_0, _hb0, MCPWM_OPR_A, duty);
}

void BMotor::setDuty1(int duty) {
    mcpwm_set_duty(MCPWM_UNIT_0, _hb1, MCPWM_OPR_A, duty);              // set half bridge duty cycle
}

void BMotor::setDuty2(int duty) {
    mcpwm_set_duty(MCPWM_UNIT_0, _hb2, MCPWM_OPR_A, duty);              // set half bridge duty cycle
}

void BMotor::setPowerLevel(int level) {                                 // set power level
    powerLevel = level;
}

void BMotor::forward() {                                                // run both motors forward
    setDuty0(powerLevel);
    setDuty1(0);
    setDuty2(powerLevel);
}

void BMotor::stop() {                                                   // stop both motors
    setDuty0(0);
    setDuty1(0);
    setDuty2(0);
}

void BMotor::reverse() {                                                // run both motors in reverse
    setDuty0(0);
    setDuty1(powerLevel);
    setDuty2(0);
}

void BMotor::right() {                                                  // make right turn
    // setDuty0(50-((float(powerLevel)/100)*50));
    setDuty0(10);
    setDuty1(50);
    setDuty2(90);
    // setDuty2(50+((float(powerLevel)/100)*50));
}

void BMotor::left() {                                                   // make left turn
    // setDuty0(50+((float(powerLevel)/100)*50));
    setDuty0(90);
    setDuty1(50);
    setDuty2(10);
    // setDuty2(50-((float(powerLevel)/100)*50));
}
