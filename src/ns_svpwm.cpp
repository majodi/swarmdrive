#include "ns_svpwm.h"

/*========================================================================*/
/*                            CONSTRUCTOR(s)                              */
/*========================================================================*/

Motor::Motor(motorConfig* MC) {
    sensorConfig sensorConfig;                                          // define config variable for RPS (Rotational Position Sensor)
    sensorConfig.i2c_gpio_sda = MC->rpsPinSDA;                          // set data pin
    sensorConfig.i2c_gpio_scl = MC->rpsPinSCL;                          // set clock pin
    sensorConfig.i2c_frequency = 600000;                                // set I2C speed (tested 600kHz to be stable)
    assert(MC->rpsResolution > 0);                                      // resolution setting of used RPS must be set (otherwise assert will reset ESP)
    _rpsResolution = MC->rpsResolution;                                 // set RPS resolution
    _rps.init(sensorConfig);                                            // init RPS
    _rps.resetAngleZero();                                              // set angle to zero
    setup_svpwm();                                                      // fill space vector array (lookup table for PWM duty cycle values)
    setup_mcpwm_pins(MC);                                               // configure mcpwm pins
    setup_mcpwm_configuration(MC->pwmFreq);                             // configure mcpwm with pwm frequency (high enough to be not audible)
    determineSignalRotationAngleR();                                    // determine motor characteristics (signal rotation repeats itself N-times depending on number of poles)
    createTimer();                                                      // prepare timer (used for exact timing of commutation)
    setStepFreq(MC->stepFreq);                                          // set period for timer to fire
}

// start motor (_running true)
void Motor::startMotor() {
    startTimer(_timerInterval);                                         // starting timer will start commutation
    _running = true;                                                    // set state
}

// stop motor (_running false)
void Motor::stopMotor() {
    if (!_running) return;                                              // if currently running timer can be stopped (otherwise we get error)
    _running = false;                                                   // set state
    stopTimer();                                                        // stop the timer (stop commutation)
    disengage();                                                        // disengage motor coils (set pwm duty to 0)
    _rpm = 0;                                                           // reset RPM
}

// set motor direction reversed
void Motor::reverseMotor() {
    _clockwise = !_clockwise;                                           // reverse speed
}

// get current direction
int Motor::getDirection() {
    return _clockwise;                                                  // return direction CW = 1 / CCW = 0
}

// get current RPM
int Motor::getRPM() {
    return _rpm;                                                        // return RPM
}

// set step frequency (commutation frequency)
void Motor::setStepFreq(int stepFreq) {
    _stepFreq = stepFreq;                                               // remember step frequency
    _timerInterval = (1.0 / _stepFreq) * 1000000;                       // calculate perion time in uS
    if (_running) {                                                     // if currently running
        stopMotor();                                                    // restart motor so that timer restarts with new value
        startMotor();
    }
}

// set torque angle
void Motor::setTorqueAngle(int angle) {
    _torqueAngle = angle;                                               // set torque angle (angle at which motor is pulled ahead, 90 should be max torque)
}

// set amplitude
void Motor::setAmplitude(int amplitude) {
    _amplitude = amplitude;                                             // set amplitude of the PWM as a percentage, this sets the power/current for the motor
}

// fill svpwm lookup table
void Motor::setup_svpwm() {
    float factor = 115.4;                                               // amplitude factor (can be more that 100% due to space vector wobble effect)
    for (int angle = 0; angle < 360; angle++) {                         // one degree resolution should be sufficient
        float a = factor*sin(RAD(angle));                               // amplitude coil a
        float b = factor*sin(RAD(angle-120));                           // amplitude coil b
        float c = factor*sin(RAD(angle+120));                           // amplitude coil c
        float g = (MIN(a,b,c)+MAX(a,b,c))/2;                            // saw tooth pattern
        float duty = ((a - g) / 2) + 57.7;                              // lift wave form to DC (positive)
        _svpwm[angle] = duty;                                           // set lookup value for degree
    }
}

// set motor at signal step
void Motor::commutate(int stepRequest) {
    
    int step = stepRequest >= _arraySize ? stepRequest - _arraySize : (stepRequest < 0 ? _arraySize + stepRequest : stepRequest);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, ((float)_amplitude / 100) * _svpwm[step]);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, ((float)_amplitude / 100) * _svpwm[step > _phaseShift ? step - _phaseShift : step + _dblPhaseShift]);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, ((float)_amplitude / 100) * _svpwm[step < _dblPhaseShift ? step + _phaseShift : step - _dblPhaseShift]);
    _lastStep = step;
}

// determine initial direction and save value
void Motor::determineDirection() {
    commutate(0);                                               // set motor at signal 0 position
    ets_delay_us(200);                                          // give it some time to settle
    _rps.resetAngleZero();                                      // reset rps angle to this position
    commutate(_arraySize / 4);                                  // move motor to 1/4 (90 deg.) position to avoid back and forth when configuration mismatch
    ets_delay_us(500000);                                       // longer delay for possible visual inspect of movement
    int startAngle = _rps.getAngleR();                          // set this position as start angle for measurement
    commutate(_arraySize - (_arraySize / 4));                   // move motor to 3/4 (270 deg.) position
    ets_delay_us(500000);                                       // give oppertunity for visual inspect
    int endAngle = _rps.getAngleR();                            // get end angle of movement
    if (endAngle > startAngle) {                                // determine direction: end angle > start so probably clockwise
        if ((endAngle - startAngle) < 120) {                    // if movement was not in excess of 1/3 for lowest resolution (360) of a one-coil-pair motor (we should have moved at least 1/2, 180 deg.)
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

// do full signal rotation and save total signal rotation angle
void Motor::determineSignalRotationAngleR() {
    assert(!_running);                                          // motor should not be running
    determineDirection();                                       // direction is needed here
    ets_delay_us(1000000);
    commutate(0);                                               // set motor at zero position
    ets_delay_us(500000);                                       // give it some time to settle (longer for visual queue)
    _rps.resetAngleZero();                                      // reset angle to zero position
    int startAngle = _rps.getAngleR();                          // record start angle
    for(int signalStep = 0; signalStep < _arraySize; signalStep++) {    // full signal rotation
        commutate(signalStep);                                          // set next position
        ets_delay_us(200);                                              // give it some time
    }
    ets_delay_us(500000);                                       // visual inspect delay
    int endAngle = _rps.getAngleR();                            // get end angle
    if (_clockwise) {                                           // if motor is miving in clockwise direction
        if (startAngle > (_rpsResolution / 2)) {                // and start angle > physical half; we started before 12 o'clock zero position
            _signalRotationAngleR = abs((float)(endAngle - startAngle));    // safe to assume end/start difference is angle
        } else {
            _signalRotationAngleR = abs((float)(_rpsResolution - endAngle + startAngle));   // else we should account for gap after 12 o'clock start position
        }
    } else {                                                    // if motor is moving in counter clockwise direction
        if (startAngle < (_rpsResolution / 2)) {                // and start angle < physical half; we started after 12 o'clock zero position
            _signalRotationAngleR = abs((float)(endAngle - startAngle));    // safe to assume end/start difference is angle
        } else {
            _signalRotationAngleR = abs((float)(_rpsResolution - startAngle + endAngle));   // else we should account for gap before 12 o'clock start position
        }
    }
    disengage();                                                // disengage coils
}

// disengage motor
void Motor::disengage() {
    float totalPWMDuty = 100;                                   // motor is fully disengaged when 0 duty percentage for all coils (assume 100)
    int retries = 10;                                           // doing retries is probably not needed here...
    while ((retries > 0) && (totalPWMDuty > 0)) {               // try to disengage (could fail perhaps, to be investigated)
        retries--;
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);    // disengage motor (duty cycle zero, no power)
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 0);
        _running = false;                                       // assume disengage successful
        totalPWMDuty = mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A) + mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A) + mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A); // what is current total duty
    }
    if (totalPWMDuty > 0) {
        printf("WARNING: PWM!!!!\n");                           // this would be NOT GOOD as it could burn driver or motor when stuck in high power
    }
}

int Motor::getSignalRotationAngle() {
    return ((float)_signalRotationAngleR / _rpsResolution) * 360;   // return physical angle (in degrees) the motor moves on one full signal cycle
}

int Motor::getAngle() {
    return ((float)_angleR / _rpsResolution) * 360;             // return current motor angle (in degrees)
}

// callback for high resolution timer
void Motor::onTimer(void *arg) {
    Motor* m = ((Motor*)arg);                                   // create pointer to motor object as this is a static method (callback can not be an instance method)
    if (!m->_running) return;                                   // needed for disengage conflict (when timer fires during disengage), see second check at commutation
#ifdef NS_TIMER_DEBUG_PIN
    gpio_set_level(NS_TIMER_DEBUG_PIN, 1);                      // signal high for timing debugging with scope
#endif
    m->_lastAngleR = m->_angleR;                                // save last angle
    m->_angleR = m->_rps.getAngleR();                           // get new (current) angle
    int delta = !m->_clockwise ?                                // calculate delta (angle at which motor has moved) - first are we moving CCW?
        (m->_lastAngleR > m->_angleR ? m->_rpsResolution - (m->_lastAngleR - m->_angleR) : m->_angleR - m->_lastAngleR ) :  // angle calculation for CCW (taking into account that rotation can pass zero point)
        (m->_lastAngleR > m->_angleR ? m->_lastAngleR - m->_angleR : m->_rpsResolution - (m->_angleR - m->_lastAngleR));    // angle calculation for CW
    m->_rpm = (m->_rpm + (m->_stepFreq * 60 * ((float)delta / m->_rpsResolution))) / 2; // scale up movement to calculate RPM (take running average to suppress noise)
    float currentSignalFraction = (float)((m->_rpsResolution - m->_angleR) % m->_signalRotationAngleR) / m->_signalRotationAngleR;      // take modulus to find position within signal rotation
    if (m->_running) m->commutate((currentSignalFraction * m->_arraySize) + (m->_clockwise ? m->_torqueAngle : -1 * m->_torqueAngle));  // commutate to next goal angle (using torque angle) only when we should be still running (not fired while disengage is called)
#ifdef NS_TIMER_DEBUG_PIN
    gpio_set_level(NS_TIMER_DEBUG_PIN, 0);                      // signal low for debugging
#endif
}

// high resolution timer setup and start
void Motor::createTimer() {
    const esp_timer_create_args_t periodic_timer_args = {       // prepare config with callback and this motor instance
        &Motor::onTimer,
        this,
        ESP_TIMER_TASK,
        "onTimer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &_periodic_timer));  // create and save handle
}

void Motor::startTimer(uint64_t interval) {
    ESP_ERROR_CHECK(esp_timer_start_periodic(_periodic_timer, interval));       // start timer with interval
}

void Motor::stopTimer() {
    ESP_ERROR_CHECK(esp_timer_stop(_periodic_timer));           // stop timer
}

// setup pins
void Motor::setup_mcpwm_pins(motorConfig* MC) {
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MC->pin0A);          // coil (a) PWM signal for driver to power motor coil
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MC->pin0B);          // driver enable signal to enable half bridge for coil (a)
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, MC->pin1A);          // coil (b) PWM signal for driver to power motor coil
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, MC->pin1B);          // driver enable signal to enable half bridge for coil (b)
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, MC->pin2A);          // coil (c) PWM signal for driver to power motor coil
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, MC->pin2B);          // driver enable signal to enable half bridge for coil (c)
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
    _running = false;
}
