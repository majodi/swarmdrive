#include "ns_svpwm.h"

/*========================================================================*/
/*                            CONSTRUCTORS                                */
/*========================================================================*/

Motor::Motor(motorConfig* MC) {
    sensorConfig sensorConfig;
    sensorConfig.i2c_gpio_sda = MC->rpsPinSDA;
    sensorConfig.i2c_gpio_scl = MC->rpsPinSCL;
    sensorConfig.i2c_frequency = 600000;
    assert(MC->rpsResolution > 0);
    _rpsResolution = MC->rpsResolution;
    _rps.init(sensorConfig);
    _rps.resetAngleZero();
    setup_mcpwm_pins(MC);
    setup_mcpwm_configuration(MC->pwmFreq);
    setSignalRotationAngle();
}

// set motor at signal step
void Motor::setPosition(int step) {
    if (step < 0 || step >= _arraySize) return;
    int a = step;
    int b = a < (_arraySize - _phaseShift) ? a + _phaseShift : a - (_arraySize - _phaseShift);
    int c = b < (_arraySize - _phaseShift) ? b + _phaseShift : b - (_arraySize - _phaseShift);
    if (_switchCoils) {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, _svpwm[a]);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, _svpwm[c]);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, _svpwm[b]);
    } else {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, _svpwm[a]);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, _svpwm[b]);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, _svpwm[c]);
    }
}

// determine initial direction and save value
void Motor::setInitialDirection() {
    setPosition(0);                                             // set motor at signal 0 position
    ets_delay_us(200);                                          // give it some time to settle
    _rps.resetAngleZero();                                      // reset rps angle to this position
    setPosition(_arraySize / 4);                                // move motor to 1/4 (90 deg.) position to avoid back and forth when configuration mismatch
    ets_delay_us(1000000);                                      // longer delay for possible visual inspect of movement
    int startAngle = _rps.getAngle();                           // set this position as start angle for measurement
    setPosition(_arraySize - (_arraySize / 4));                 // move motor to 3/4 (270 deg.) position
    ets_delay_us(1000000);                                      // give oppertunity for visual inspect
    int endAngle = _rps.getAngle();                             // get end angle of movement
    if (endAngle > startAngle) {                                // determine direction: end angle > start so probably clockwise
        if ((endAngle - startAngle) < 120) {                    // if movement was not in excess of 1/3 for lowest resolution of a one-coil-pair motor (we should have moved at least 1/2, 180 deg.)
            _clockwise = true;                                  // then safe to say we moved clockwise, set clockwise to true
        } else {
            _clockwise = false;                                 // else we moved counter clockwise, set clockwise to false
        }
    } else {                                                    // end angle < start so probably counter clockwise
        if ((startAngle - endAngle) < 120) {                    // if movement was not in excess of 1/3 for lowest resolution of a one-coil-pair motor (we should have moved at least 1/2, 180 deg.)
            _clockwise = false;                                 // then safe to say we moved counter clockwise, set clockwise to false
        } else {
            _clockwise = true;                                  // else we moved clockwise, set clockwise to true
        }
    }
}

// for measurement purposes
void Motor::trySetSignalRotationAngle() {
    assert(!_running);                                          // motor should not be running
    _rps.resetAngleZero();                                      // reset angle to zero position
    ets_delay_us(1000);                                          // give it some time to settle
    int startAngle = _rps.getAngle();                           // record start angle
    for(int signalStep = 0; signalStep < _arraySize; signalStep++) {    // full signal rotation
        setPosition(signalStep);                                        // set next position
        ets_delay_us(200);                                              // give it some time
    }
    ets_delay_us(500000);                                       // visual inspect delay
    int endAngle = _rps.getAngle();                             // get end angle
    if (_clockwise) {                                           // if motor is miving in clockwise direction
        if (startAngle > (_rpsResolution / 2)) {                // and start angle > physical half; we started before 12 o'clock zero position
            _signalRotationAngle = abs(((float)(endAngle - startAngle) / _rpsResolution) * _arraySize);                     // safe to assume end/start difference is angle
        } else {
            _signalRotationAngle = abs(((float)(_rpsResolution - endAngle + startAngle) / _rpsResolution) * _arraySize);    // else we should account for gap after 12 o'clock start position
        }
    } else {                                                    // if motor is moving in counter clockwise direction
        if (startAngle < (_rpsResolution / 2)) {                // and start angle < physical half; we started after 12 o'clock zero position
            _signalRotationAngle = abs(((float)(endAngle - startAngle) / _rpsResolution) * _arraySize);                     // safe to assume end/start difference is angle
        } else {
            _signalRotationAngle = abs(((float)(_rpsResolution - startAngle + endAngle) / _rpsResolution) * _arraySize);    // else we should account for gap before 12 o'clock start position
        }
    }
}

// disengage motor
void Motor::disengage() {
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);    // disengage motor (duty cycle zero, no power)
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 0);
}

// do full signal rotation and save total signal rotation angle
void Motor::setSignalRotationAngle() {
    assert(!_running);                                          // motor should not be running
    setInitialDirection();                                      // direction is needed here
    setPosition(0);                                             // set motor at zero position
    ets_delay_us(500000);                                      // give it some time to settle (longer for visual queue)
    trySetSignalRotationAngle();                                // set signal rotation angle with current config
    if ((_signalRotationAngle > 270) || (_signalRotationAngle < 6)) {   // if angle outcome is to small or to big (for a multi coil-pair motor) the motor probable moved back and forth unexpected
        _switchCoils = true;                                    // try to fix this with a virtual coil switch
        _clockwise = !_clockwise;                               // and invert the determined direction accordingly
        trySetSignalRotationAngle();                            // second attempt
        assert(!(_signalRotationAngle > 270) || (_signalRotationAngle < 6));    // assert on fail second attempt
        trySetSignalRotationAngle();                            // make final measurement
        assert(!(_signalRotationAngle > 270) || (_signalRotationAngle < 6));    // assert on fail final measurement
    }
    _rps.resetAngleZero();                                      // reset angle
    ets_delay_us(500000);                                       // visual queue for end of measurements
    disengage();                                                // disengage coils
}

int Motor::getSignalRotationAngle() {
    return _signalRotationAngle;
}

int Motor::getAngle() {
    return _angle;
}

// callback for high resolution timer
void Motor::onTimer(void *arg) {
    Motor* m = ((Motor*)arg);
    m->_lastAngle = m->_angle;
    m->_angle = m->_rps.getAngle();
#ifdef NS_TIMER_DEBUG_PIN
    gpio_set_level(NS_TIMER_DEBUG_PIN, 1);
#endif
    // set next step based on current angle and direction (90 degrees for maximum torque)
    // m->stepA = (((motor->angle % motor->signalRotationAngle) * motor->arraySize) / motor->signalRotationAngle) + (motor->forward ? 90 : -90);
    // motor->stepB = motor->stepA + motor->phaseShift;
    // motor->stepC = motor->stepB + motor->phaseShift;

    //     mcpwmParams* motor = (mcpwmParams*)arg;
    //     motor->lastAngle = motor->angle;
    //     motor->angle = motor->rps.angleR(U_DEG);
    //     // set next step based on current angle and direction (90 degrees for maximum torque)
    //     motor->stepA = (((motor->angle % motor->signalRotationAngle) * motor->arraySize) / motor->signalRotationAngle) + (motor->forward ? 90 : -90);
    //     motor->stepB = motor->stepA + motor->phaseShift;
    //     motor->stepC = motor->stepB + motor->phaseShift;
    //     // adjust for (over/under)flow (next step)
    //     motor->stepA = motor->stepA < 0 ? motor->arraySize-1 + motor->stepA : (motor->stepA > motor->arraySize-1 ? 0 + motor->stepA - motor->arraySize-1 : motor->stepA);
    //     motor->stepB = motor->stepB < 0 ? motor->arraySize-1 + motor->stepB : (motor->stepB > motor->arraySize-1 ? 0 + motor->stepB - motor->arraySize-1 : motor->stepB);
    //     motor->stepC = motor->stepC < 0 ? motor->arraySize-1 + motor->stepC : (motor->stepC > motor->arraySize-1 ? 0 + motor->stepC - motor->arraySize-1 : motor->stepC);
    //     // set SVPWM duty cycle percentage
    //     // printf("current angle: %d - next step: %d, %d, %d\n", motor->angle, motor->stepA, motor->stepB, motor->stepC);
    //     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, motor->running ? motor->svpwm[motor->stepA] : 0);
    //     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, motor->running ? motor->svpwm[motor->stepB] : 0);
    //     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, motor->running ? motor->svpwm[motor->stepC] : 0);
#ifdef NS_TIMER_DEBUG_PIN
    gpio_set_level(NS_TIMER_DEBUG_PIN, 0);
#endif
}

// high resolution timer setup and start
void Motor::createTimer() {
    const esp_timer_create_args_t periodic_timer_args = {
        &Motor::onTimer,
        this,
        ESP_TIMER_TASK,
        "onTimer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &_periodic_timer));
}

void Motor::startTimer(uint64_t interval) {
    ESP_ERROR_CHECK(esp_timer_start_periodic(_periodic_timer, interval));
}

void Motor::stopTimer() {
    ESP_ERROR_CHECK(esp_timer_stop(_periodic_timer));
}

// setup pins
void Motor::setup_mcpwm_pins(motorConfig* MC) {
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MC->pin0A);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MC->pin0B);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, MC->pin1A);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, MC->pin1B);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, MC->pin2A);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, MC->pin2B);
#ifdef NS_TIMER_DEBUG_PIN
        gpio_pad_select_gpio(NS_TIMER_DEBUG_PIN);                  // debug pin as a gpio function
        gpio_set_direction(NS_TIMER_DEBUG_PIN, GPIO_MODE_OUTPUT);  // set debug pin as output
#endif
}

// setup mcpwm configuration
void Motor::setup_mcpwm_configuration(int pwmFreq) {
    mcpwm_config_t pwm_config;
    pwm_config.frequency = pwmFreq;                         // up-down counter results in half of this
    pwm_config.cmpr_a = 50.0;                               // duty cycle of PWMxA = 50.0%
    pwm_config.cmpr_b = 50.0;                               // duty cycle of PWMxB = 50.0%
    pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER;        // Up-down counter (triangle wave)
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;               // Active HIGH
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);   // Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);   // Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);   // Configure PWM0A & PWM0B with above settings
    mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_SELECT_TIMER0, 0); // !!! MCPWM_SELECT_TIMER1 didn't exist in mcpwm.h, had to edit the header
    mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_SELECT_TIMER0, 0); // it sets to sync all timers to timer0
    mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_SELECT_TIMER0, 0); // see Github issue https://github.com/espressif/esp-idf/issues/5429
    MCPWM0.timer[0].sync.out_sel = 1;                       // timer 0 as sync input (https://github.com/espressif/esp-idf/issues/3567)
    vTaskDelay(500/portTICK_PERIOD_MS);                     // wait long enough for large period PWM (low frequency) to be able to sysnc
    MCPWM0.timer[0].sync.out_sel = 0;                       // done syncing
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);    // set initial duty 0 (disengage motor)
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 100);  // set enable to continues high
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 100);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, 100);
}

// setup Rotation Position Sensor
// void setup_rps() {
//     Wire.begin(_NS_RPS_WIRE_SDA, _NS_RPS_WIRE_SCL);             // start wire for I2C communication rps
// }
    // // initialize motor values
    // static mcpwmParams initMotor() {
    //     mcpwmParams motor;
    //     initRPS();
    //     motor.rps.begin();                                          // start rps clockwise and reset to zero
    //     motor.rps.setClockWise(true);
	//     motor.rps.setZeroReg();
    //     motor.lastAngle         = 0;                                // initialize lastknown angle
    //     motor.lastRunningState   = false;                           // initialize last running state
    //     motor.stepA             = 0;                                // initialize step ABC
    //     motor.stepB             = motor.stepA + motor.phaseShift;
    //     motor.stepC             = motor.stepB + motor.phaseShift;
    //     motor.running           = false;                            // initial running state
    //     motor.forward           = _NS_DEF_DIRECTION;                // initial direction
    //     motor.stepFreq          = _NS_DEF_STEP_FREQ;                // initial step frequency
    //     motor.angle             = motor.rps.angleR(U_DEG);          // motor angle
    //     motor.turns             = 0;                                // motor number of turns from start
    //     motor.turnTime          = 0;                                // time stamp (uS) of last full turn
    //     motor.turnDuration      = 0;                                // time (uS) of last full turn
    //     motor.consoleUpdate     = esp_timer_get_time();             // set last console update time to now
    //     setSignalRotationAngle(motor);
    //     return motor;
    // }

    // // setup ESP32 mcpwm
    // static void setup_mcpwm(mcpwmConfig taskConfig) {
    //     setup_mcpwm_pins(taskConfig);
    //     mcpwm_config_t pwm_config;
    //     pwm_config.frequency = 40000;                           // up-down counter results in half of this
    //     pwm_config.cmpr_a = 50.0;                               // duty cycle of PWMxA = 50.0%
    //     pwm_config.cmpr_b = 50.0;                               // duty cycle of PWMxB = 50.0%
    //     pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER;        // Up-down counter (triangle wave)
    //     pwm_config.duty_mode = MCPWM_DUTY_MODE_0;               // Active HIGH
    //     mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);   // Configure PWM0A & PWM0B with above settings
    //     mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);   // Configure PWM0A & PWM0B with above settings
    //     mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);   // Configure PWM0A & PWM0B with above settings
    //     mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_SELECT_SYNC_OUT0, 0); // !!! MCPWM_SELECT_SYNC_OUT0 didn't exist in mcpwm.h, had to edit the header
    //     mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_SELECT_SYNC_OUT0, 0); // it sets to sync all timers to timer0
    //     mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_SELECT_SYNC_OUT0, 0); // newer versions of esp-idf do have new (more) enum values
    //     MCPWM0.timer[0].sync.out_sel = 1;                       // timer 0 as sync input (https://github.com/espressif/esp-idf/issues/3567)
    //     delayMicroseconds(100 / portTICK_PERIOD_MS);
    //     MCPWM0.timer[0].sync.out_sel = 0;
    //     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);    // set initial duty 0 (disengage motor)
    //     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 100);  // set enable high
    //     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
    //     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 100);
    //     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 0);
    //     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, 100);
    // }

    // // check if console has command or parameter setting
    // static void checkMessages(mcpwmParams &motor, esp_timer_handle_t periodic_timer) {
    //     consoleMessageStruct consoleMessage = availableConsoleMessage();                    // see if there are new messages from console
    //     if (consoleMessage.messageType != 0) {                                              // if known message type
    //         if (consoleMessage.messageType == _NS_COMMAND) {                                // *** if command
    //             if (consoleMessage.identifier == _NS_C_START) motor.running = true;         // and command is START the signal running to rest of code here
    //             else if (consoleMessage.identifier == _NS_C_STOP) {                         // if STOP
    //                 motor.running = false;                                                  // signal not running
    //                 sendToConsole(_NS_SET_PARAMETER, _NS_P_RPM, 0);                         // ask console to set RPM to zero
    //             }
    //             else if (consoleMessage.identifier == _NS_C_REVERSE) {                      // if REVERSE
    //                 motor.forward = !motor.forward;                                         // reverse direction
    //                 sendToConsole(_NS_SET_PARAMETER, _NS_P_DIRECTION, motor.forward);       // ask console to let user know (display new state)
    //             }
    //         }
    //         else if (consoleMessage.messageType == _NS_SET_PARAMETER) {                     // *** if parameter change
    //             if (consoleMessage.identifier == _NS_P_DIRECTION) {                         // direction change
    //                 if (consoleMessage.value == 0 || consoleMessage.value == 1) {           // if can only be forward or backward
    //                     motor.forward = consoleMessage.value;                               // set direction
    //                 } else {
    //                     consoleLogMessage("dir. error");                                    // else let console log error message
    //                     sendToConsole(_NS_SET_PARAMETER, _NS_P_DIRECTION, motor.forward);   // set direction to forward
    //                 }
    //             }
    //             if (consoleMessage.identifier == _NS_P_STEP_FREQ) {                         // step frequency change
    //                 motor.stepFreq = consoleMessage.value;                                  // set new step frequency
    //                 if (motor.running) {                                                    // if motor running; stop timer, start timer with new step time interval
    //                     ESP_ERROR_CHECK(esp_timer_stop(periodic_timer));                    
    //                     ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, (1.0 / motor.stepFreq) * 1000000));
    //                 }
    //             }
    //         }
    //     }
    // }

    // // advance the motor (callback for high resolution timer)
    // static void advanceMotor(void *arg) {
    //     #ifdef _NS_TIMER_DEBUG_PIN
    //     gpio_set_level(_NS_TIMER_DEBUG_PIN, 1);
    //     #endif
    //     mcpwmParams* motor = (mcpwmParams*)arg;
    //     motor->lastAngle = motor->angle;
    //     motor->angle = motor->rps.angleR(U_DEG);
    //     // set next step based on current angle and direction (90 degrees for maximum torque)
    //     motor->stepA = (((motor->angle % motor->signalRotationAngle) * motor->arraySize) / motor->signalRotationAngle) + (motor->forward ? 90 : -90);
    //     motor->stepB = motor->stepA + motor->phaseShift;
    //     motor->stepC = motor->stepB + motor->phaseShift;
    //     // adjust for (over/under)flow (next step)
    //     motor->stepA = motor->stepA < 0 ? motor->arraySize-1 + motor->stepA : (motor->stepA > motor->arraySize-1 ? 0 + motor->stepA - motor->arraySize-1 : motor->stepA);
    //     motor->stepB = motor->stepB < 0 ? motor->arraySize-1 + motor->stepB : (motor->stepB > motor->arraySize-1 ? 0 + motor->stepB - motor->arraySize-1 : motor->stepB);
    //     motor->stepC = motor->stepC < 0 ? motor->arraySize-1 + motor->stepC : (motor->stepC > motor->arraySize-1 ? 0 + motor->stepC - motor->arraySize-1 : motor->stepC);
    //     // set SVPWM duty cycle percentage
    //     // printf("current angle: %d - next step: %d, %d, %d\n", motor->angle, motor->stepA, motor->stepB, motor->stepC);
    //     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, motor->running ? motor->svpwm[motor->stepA] : 0);
    //     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, motor->running ? motor->svpwm[motor->stepB] : 0);
    //     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, motor->running ? motor->svpwm[motor->stepC] : 0);
    //     #ifdef _NS_TIMER_DEBUG_PIN
    //     gpio_set_level(_NS_TIMER_DEBUG_PIN, 0);
    //     #endif
    // }

    // // high resolution timer setup and start
    // static void startTimer(mcpwmParams &motor, esp_timer_handle_t &periodic_timer) {
    //     const esp_timer_create_args_t periodic_timer_args = {       // prepare timer args
    //         &advanceMotor,                                          // callback
    //         &motor,                                                 // callback arg
    //         ESP_TIMER_TASK,                                         // method callback from timer task
    //         "advanceTimer"                                          // descriptive name
    //     };
    //     ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer)); // create timer
    // }

    // #ifdef _NS_POT_PIN
    // // read pot value
    // static void readPot(int &lastPotValue, int &potSteadyCnt, mcpwmParams &motor, esp_timer_handle_t periodic_timer) {
    //     int potValue = analogRead(_NS_POT_PIN);                     // read pot
    //     if (abs(potValue - lastPotValue) > 20) {                    // debounce if pot (change was significant enough)
    //         motor.stepFreq = potValue * _NS_POT_FACTOR;             // set step freq if pot is used (with factor)
    //         lastPotValue = potValue;                                // remember pot value
    //         potSteadyCnt = 1;                                       // arm steady counter (counting cycles before updating console too quickly)
    //         if (motor.running) {                                    // if motor running; stop timer, start timer with new step period
    //             ESP_ERROR_CHECK(esp_timer_stop(periodic_timer));
    //             ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, (1.0 / motor.stepFreq) * 1000000));
    //         }
    //     } else {
    //         potSteadyCnt = potSteadyCnt > 0 ? potSteadyCnt + 1 : potSteadyCnt;      // update steady count
    //         if (potSteadyCnt > 25) {                                                // after some time
    //             sendToConsole(_NS_SET_PARAMETER, _NS_P_STEP_FREQ, motor.stepFreq);  // update console with new (steady) value
    //             potSteadyCnt = 0;                                                   // stop counting for this value
    //         }
    //     }
    // }
    // #endif

    // // check for full turn and update console
    // static void checkFullTurn(mcpwmParams &motor) {
    //     if ((motor.forward && (motor.angle < motor.lastAngle)) || (!motor.forward && (motor.angle > motor.lastAngle))) {
    //         int64_t currentTime = esp_timer_get_time();             // get current time (uS)
    //         motor.turnDuration = currentTime - motor.turnTime;      // calculate period time with last known turn time (uS)
    //         motor.turnTime = currentTime;                           // set current time as last turn time (uS)
    //         motor.turns++;                                          // increment number of turns
    //         if ((currentTime - motor.consoleUpdate) > 1000000) {    // update console about once per second
    //             sendToConsole(_NS_SET_PARAMETER, _NS_P_RPM, 60 / ((double)motor.turnDuration / 1000000));
    //             motor.consoleUpdate = currentTime;
    //         }
    //     }
    // }

    // // set motor at signal step
    // static void setPosition(mcpwmParams &motor, int step) {
    //     if (step < 0 || step >= motor.arraySize) return;
    //     int a = step;
    //     int b = a < (motor.arraySize - motor.phaseShift) ? a + motor.phaseShift : a - (motor.arraySize - motor.phaseShift);
    //     int c = b < (motor.arraySize - motor.phaseShift) ? b + motor.phaseShift : b - (motor.arraySize - motor.phaseShift);
    //     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, motor.svpwm[a]);
    //     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, motor.svpwm[b]);
    //     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, motor.svpwm[c]);
    // }

    // // wiggle motor a little to settle position
    // static void wiggle(mcpwmParams &motor) {
    //     setPosition(motor, 0);
    //     vTaskDelay(1);
    //     setPosition(motor, motor.arraySize / (motor.arraySize / 10));
    //     vTaskDelay(1);
    //     setPosition(motor, 0);
    //     vTaskDelay(1);
    //     setPosition(motor, motor.arraySize / (motor.arraySize / 10));
    //     vTaskDelay(1);
    //     setPosition(motor, 0);
    //     vTaskDelay(1);
    // }

    // // disengage motor
    // static void disengage() {
    //     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);    // disengage motor (duty cycle zero, no power)
    //     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
    //     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 0);
    // }

    // static void setSignalRotationAngle(mcpwmParams &motor) {
    //     wiggle(motor);                                              // wiggle to settle position
	//     motor.rps.setZeroReg();                                     // reset angle
    //     ets_delay_us(200);
    //     for(int sd = 0; sd < motor.arraySize; sd++) {               // full signal rotation
    //         setPosition(motor, sd);                                 // set position
    //         ets_delay_us(200);
    //     }
    //     disengage();
    //     motor.signalRotationAngle = (int)(motor.rps.angleR(U_DEG));
	//     motor.rps.setZeroReg();                                     // reset angle
    // }

    // register commands and parameters
    // static void consoleRegistration() {
    //     sendToConsole(_NS_REG_COMMAND, _NS_C_START, 0, "r", "Run Motor"); // register console commands/parameters
    //     sendToConsole(_NS_REG_COMMAND, _NS_C_STOP, 0, "s", "Stop Motor");
    //     sendToConsole(_NS_REG_COMMAND, _NS_C_REVERSE, 0, "rv", "Reverse");
    //     sendToConsole(_NS_REG_PARAMETER, _NS_P_STEP_FREQ, _NS_DEF_STEP_FREQ, "sf", "Step Freq");
    //     sendToConsole(_NS_REG_PARAMETER, _NS_P_DIRECTION, _NS_DEF_DIRECTION, "d", "Direction");
    //     sendToConsole(_NS_REG_PARAMETER, _NS_P_RPM, 0, "--", "RPM");
    // }

    // // reset motor angle
    // static void resetMotorAngle() {

    // }

    // // init rotation position sensor
    // static bool initRPS() {
    //     Wire.begin(_NS_RPS_WIRE_SDA, _NS_RPS_WIRE_SCL);             // start wire for I2C communication rps
    // }

    // // get motor angle
    // static double getMotorAngle() {

    // }

    // main task
    // void mcpwmTask(void *arg) {
    //     mcpwmConfig* taskConfig = (mcpwmConfig *) arg;
    //     esp_timer_handle_t periodic_timer;                          // timer handle
    //     if (taskConfig->potPin != GPIO_NUM_NC) {
    //         int lastPotValue = 0;                                       // initialize last pot value
    //         int potSteadyCnt = 0;                                       // initialize pot steady counter (0 is undefined >0 is cycles steady (same) value)
    //     }
    //     if (taskConfig->debugPin != GPIO_NUM_NC) {
    //         gpio_pad_select_gpio(taskConfig->debugPin);                  // debug pin as a gpio function
    //         gpio_set_direction(taskConfig->debugPin, GPIO_MODE_OUTPUT);  // set debug pin as output
    //     }
    //     consoleRegistration();                                      // register commands and parameters
    //     // setup_mcpwm(*taskConfig);                                   // setup mcpwm unit
    //     // mcpwmParams motor = initMotor();                            // initialize motor
    //     // startTimer(motor, periodic_timer);                          // start the timer
    //     // while(1) {
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
    //     //     vTaskDelay(1);
    //     // }
    // }

