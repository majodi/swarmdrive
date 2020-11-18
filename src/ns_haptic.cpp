#include "ns_haptic.h"

/*========================================================================*/
/*                            CONSTRUCTOR(s)                              */
/*========================================================================*/

HRE::HRE(motorConfig* MC) {
    sensorConfig sensorConfig;                                          // define config variable for RPS (Rotational Position Sensor)
    sensorConfig.i2c_gpio_sda = MC->rpsPinSDA;                          // set data pin
    sensorConfig.i2c_gpio_scl = MC->rpsPinSCL;                          // set clock pin
    sensorConfig.i2c_frequency = 600000;                                // set I2C speed (tested 600kHz to be stable)
    sensorConfig.rpsFrontMount = MC->rpsFrontMount;                     // set mount type (where sensor is mounted)
    assert(MC->rpsResolution > 0);                                      // resolution setting of used RPS must be set (otherwise assert will reset ESP)
    _rpsResolution = MC->rpsResolution;                                 // set RPS resolution
    _rps.init(sensorConfig);                                            // init RPS
    _rps.resetAngleZero();                                              // set angle to zero
    setup_sinepwm();                                                    // fill space vector array (lookup table for PWM duty cycle values)
    setup_mcpwm_pins(MC);                                               // configure mcpwm pins
    setup_mcpwm_configuration(MC->pwmFreq);                             // configure mcpwm with pwm frequency (high enough to be not audible)
    createTimer();                                                      // prepare timer (used for position sampling)
    startTimer((1.0 / MC->sampleFreq) * 1000000);                       // start timer with sampling period in uS
    _sampleFreq = MC->sampleFreq;                                       // save freq setting
    _powerLevel = MC->powerLevel;                                       // save power level
    _repeatAngleR = ((float)MC->repeatAngleDeg / 360) * _rpsResolution;        // set repeat angleR
    _rps.resetAngleZero();                                              // reset angle to zero position
    _angleR = _rps.getAngleR();                                         // get angle after zeroing
    _lastAngleR = _angleR;                                              // last angle is current
}

/*========================================================================*/
/*                            PUBLIC METHODS                              */
/*========================================================================*/

// // set motor direction reversed
// // onlyPoles: use true to correct motor direction without reversing sensor
// void HRE::reverseMotor(bool onlyPoles) {
//     mcpwm_timer_t saveValue = _coil0;                                   // save coil0
//     _coil0 = _coil1;                                                    // switch coil0 - coil1
//     _coil1 = saveValue;
//     if (!onlyPoles) {                                                   // if NOT only poles switch
//         _clockwise = !_clockwise;                                       // remember state (register this state change)
//         _rps.invertReadings();                                          // invert readings so we still get incremental (positive) readings
//     }
// }

// // disengage motor
// void HRE::disengage() {
//     float totalPWMDuty = 100;                                           // motor is fully disengaged when 0 duty percentage for all coils (assume 100)
//     int retries = 10;                                                   // doing retries is probably not needed here...
//     while ((retries > 0) && (totalPWMDuty > 0)) {                       // try to disengage (could fail perhaps, to be investigated)
//         retries--;                                                      // one retry
//         mcpwm_set_duty(MCPWM_UNIT_0, _coil0, MCPWM_OPR_A, 0);           // disengage motor (duty cycle zero, no power)
//         mcpwm_set_duty(MCPWM_UNIT_0, _coil1, MCPWM_OPR_A, 0);
//         mcpwm_set_duty(MCPWM_UNIT_0, _coil2, MCPWM_OPR_A, 0);
//         totalPWMDuty = mcpwm_get_duty(MCPWM_UNIT_0, _coil0, MCPWM_OPR_A) + mcpwm_get_duty(MCPWM_UNIT_0, _coil1, MCPWM_OPR_A) + mcpwm_get_duty(MCPWM_UNIT_0, _coil2, MCPWM_OPR_A); // what is current total duty
//     }
//     assert(totalPWMDuty == 0);                                          // this should not happen, can overheat coils and controller
// }

// get current rotary angle in degrees
int HRE::getAngle() {
    // no need to refresh the angle, timer will run continues
    return ((float)_angleR / _rpsResolution) * 360;                     // return current motor angle (in degrees)
}

// get current movement speed
int HRE::getSpeed() {
    return (_avgDelta * 360 * _sampleFreq) / _rpsResolution;            // return deg/s
}

// get current movement direction
int HRE::getDirection() {
    return _dir;                                                        // return direction
}

// set sample frequency
void HRE::setSampleFreq(int sampleFreq) {
    _sampleFreq = sampleFreq;                                           // remember new sample frequency
    stopTimer();                                                        // stop timer
    startTimer((1.0 / _sampleFreq) * 1000000);                          // start timer with sampling period in uS
}

// set power level
void HRE::setPowerLevel(int power) {
    _powerLevel = power;
}

// set repeat angle in degrees
void HRE::setRepeatAngle(int degAngle) {
    _repeatAngleR = ((float)degAngle / 360) * _rpsResolution;
}

void HRE::handle(int angle) {
    if (_dir == 1) {
        while(abs(getAngle() - angle) < 5){
            commutate(_lastStep-25);
            ets_delay_us(SST);                                                  // let it settle
        }
    } else {
        while(abs(angle - getAngle()) < 5){
            commutate(_lastStep+25);
            ets_delay_us(SST);                                                  // let it settle
        }
    }
    mcpwm_set_duty(MCPWM_UNIT_0, _coil0, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, _coil1, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, _coil2, MCPWM_OPR_A, 0);
}

/*========================================================================*/
/*                            PRIVATE METHODS                             */
/*========================================================================*/

// setup pins
void HRE::setup_mcpwm_pins(motorConfig* MC) {
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MC->pin0A);                  // half bridge 0 high side
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MC->pin0B);                  // driver enable signal to enable half bridge 0 low side
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, MC->pin1A);                  // half bridge high side 1
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, MC->pin1B);                  // driver enable signal to enable half bridge 1 low side
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, MC->pin2A);                  // half bridge high side 2
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, MC->pin2B);                  // driver enable signal to enable half bridge 2 low side
}

// setup mcpwm configuration
void HRE::setup_mcpwm_configuration(int pwmFreq) {
    mcpwm_config_t pwm_config;                                          // mcpwm config parameter
    pwm_config.frequency = pwmFreq * 2;                                 // up-down counter uses two half cycles so for full cycle to have desired freq it has to be doubled
    pwm_config.cmpr_a = 50.0;                                           // duty cycle of PWMxA = 50.0% (temporary)
    pwm_config.cmpr_b = 50.0;                                           // duty cycle of PWMxB = 50.0% (temporary)
    // pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER;                    // Up-down counter (triangle wave for svpwm)
    pwm_config.counter_mode = MCPWM_UP_COUNTER;                         // Up counter (for sine approach)
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;                           // Active HIGH
    mcpwm_init(MCPWM_UNIT_0, _coil0, &pwm_config);                      // Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_0, _coil1, &pwm_config);                      // Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_0, _coil2, &pwm_config);                      // Configure PWM0A & PWM0B with above settings
    mcpwm_sync_enable(MCPWM_UNIT_0, _coil0, MCPWM_SELECT_TIMER0, 0);    // !!! MCPWM_SELECT_TIMER didn't exist in mcpwm.h, had to edit the header
    mcpwm_sync_enable(MCPWM_UNIT_0, _coil1, MCPWM_SELECT_TIMER0, 0);    // it sets to sync all timers to timer0
    mcpwm_sync_enable(MCPWM_UNIT_0, _coil2, MCPWM_SELECT_TIMER0, 0);    // see Github issue https://github.com/espressif/esp-idf/issues/5429
    MCPWM0.timer[0].sync.out_sel = 1;                                   // timer 0 as sync input (https://github.com/espressif/esp-idf/issues/3567)
    vTaskDelay(500/portTICK_PERIOD_MS);                                 // wait long enough for large period PWM (low frequency) to be able to sysnc
    MCPWM0.timer[0].sync.out_sel = 0;                                   // done syncing
    mcpwm_set_duty(MCPWM_UNIT_0, _coil0, MCPWM_OPR_A, 0);               // set initial duty 0 (disengage motor)
    mcpwm_set_duty(MCPWM_UNIT_0, _coil0, MCPWM_OPR_B, 100);             // set enable (low side of half bridge) to continues High, will switch low due to inverter in L6234 when high side is high
    mcpwm_set_duty(MCPWM_UNIT_0, _coil1, MCPWM_OPR_A, 0);               // same for hb
    mcpwm_set_duty(MCPWM_UNIT_0, _coil1, MCPWM_OPR_B, 100);             // same for hb
    mcpwm_set_duty(MCPWM_UNIT_0, _coil2, MCPWM_OPR_A, 0);               // same for hb
    mcpwm_set_duty(MCPWM_UNIT_0, _coil2, MCPWM_OPR_B, 100);             // same for hb
}

// fill sine lookup table
void HRE::setup_sinepwm() {
    for (int i = 0; i < SR; i++) {                                      // one degree resolution should be sufficient but can be controlled with SR (Signal Resolution)
        float angle = ((float)i / SR) * 360;                            // calculate angle for array index
        float duty = (sin(RAD(angle)) * 50.0) + 50;                     // calculate duty
        _sine[i] = duty;                                                // set lookup value for step index
    }
}

// high resolution timer setup and start
void HRE::createTimer() {
    const esp_timer_create_args_t periodic_timer_args = {               // prepare config with callback and this motor instance
        &HRE::onTimer,
        this,
        ESP_TIMER_TASK,
        "onTimer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &_periodic_timer));  // create and save handle
}

// start timer
void HRE::startTimer(uint64_t interval) {
    ESP_ERROR_CHECK(esp_timer_start_periodic(_periodic_timer, interval));       // start timer with interval
}

// stop timer
void HRE::stopTimer() {
    ESP_ERROR_CHECK(esp_timer_stop(_periodic_timer));                   // stop timer
}

// home motor; commutate to 0-position and reset angle
void HRE::home() {
    commutate(0);                                                       // commutate to 0-position
    ets_delay_us(SST);                                                  // let it settle
    _rps.resetAngleZero();                                              // reset angle
    _angleR = _rps.getAngleR();                                         // reset angle property
    _lastAngleR = _angleR;                                              // reset last angle property
}

// callback for high resolution timer
void HRE::onTimer(void *arg) {
    HRE* h = ((HRE*)arg);                                               // create pointer to motor object as this is a static method (callback can not be an instance method)
    h->_lastAngleR = h->_angleR;                                        // save last angle
    h->_angleR = h->_rps.getAngleR();                                   // get new (current) angle
    if (h->_sampleCnt == 100) {                                         // after 100 samples take agv delta and determine correct direction
        h->_dir = h->_angleR > h->_multiSampleStartAngle ? 1 : -1;      // determine direction
        h->_avgDelta = round((float)(h->_dir * (h->_angleR - h->_multiSampleStartAngle)) / 100);    // determine avg delta
        h->_sampleCnt = 0;                                              // reset sample counter
        h->_multiSampleStartAngle = h->_angleR;                         // reset start angle
    } else {
        h->_sampleCnt++;                                                // count
    }
}

// set motor at signal step
void HRE::commutate(int stepRequest) {
    int step = stepRequest >= SR ? stepRequest - SR : (stepRequest < 0 ? SR + stepRequest : stepRequest);   // adjust to signal array boundary
    if ((step >= SR) || step < 0) return;                                                                   // leave if that didn't work (request out of range)
    mcpwm_set_duty(MCPWM_UNIT_0, _coil0, MCPWM_OPR_A, ((float)_powerLevel / 100) * _sine[step]);            // set signal values
    mcpwm_set_duty(MCPWM_UNIT_0, _coil1, MCPWM_OPR_A, ((float)_powerLevel / 100) * _sine[step >= _phaseShift ? step - _phaseShift : step + _dblPhaseShift]);
    mcpwm_set_duty(MCPWM_UNIT_0, _coil2, MCPWM_OPR_A, ((float)_powerLevel / 100) * _sine[step < _dblPhaseShift ? step + _phaseShift : step - _dblPhaseShift]);
    _lastStep = step;                                                                                       // remember last step
}
