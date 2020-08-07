#include "ns_svpwm.h"

/*========================================================================*/
/*                            CONSTRUCTOR(s)                              */
/*========================================================================*/

Motor::Motor(motorConfig* MC) {
    sensorConfig sensorConfig;                                          // define config variable for RPS (Rotational Position Sensor)
    sensorConfig.i2c_gpio_sda = MC->rpsPinSDA;                          // set data pin
    sensorConfig.i2c_gpio_scl = MC->rpsPinSCL;                          // set clock pin
    sensorConfig.i2c_frequency = 600000;                                // set I2C speed (tested 600kHz to be stable)
    sensorConfig.rpsFrontMount = MC->rpsFrontMount;                     // set mount type (where sensor is mounted)
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
    setTorqueAngle(MC->torqueAngle);                                    // set torque angle
    setAmplitude(MC->amplitude);                                        // set amplitude
    setMoveSteps(MC->moveSteps);                                        // set move steps
    _rps.resetAngleZero();                                              // reset angle to zero position
    _angleR = _rps.getAngleR();                                         // get angle after zeroing
    _lastAngleR = _angleR;                                              // last angle is current
    _initialized = true;                                                // signal motor initialized
}

/*========================================================================*/
/*                            PUBLIC METHODS                              */
/*========================================================================*/

// get motor initialized status
bool Motor::isInitialized() {
    return _initialized;                                                // return initialized state
}

// return running state
bool Motor::isRunning() {
    return _running;
}

// start motor (_running true)
void Motor::startMotor() {
    home();                                                             // home and reset angle
    startTimer(_timerInterval);                                         // starting timer will start commutation
    _running = true;                                                    // set state
    _di = 0;                                                            // reset debug index on start of fresh run
}

// stop motor (_running false) and if needed make debug dump
void Motor::stopMotor() {
    if (!_running) return;                                              // no stopping if not running
    _running = false;                                                   // set state
    stopTimer();                                                        // stop the timer (stop commutation)
    disengage();                                                        // disengage motor coils (set pwm duty to 0)
    _delta = 0;                                                         // reset delta movement
    if (_debugRun) {                                                    // if debugRun is/was active
        // *** example debug code (see onTimer code for counterpart) ***
        for (int i = 0; i < 198; i=i+3)                                 // dump values
        {
            int __lastAngleR = _debug[i];
            int __angleR = _debug[i+1];
            int __delta = __angleR - __lastAngleR;
            int __sector = __angleR / _signalRotationAngleR;
            int __insect = __angleR % _signalRotationAngleR;
            float __factor = (float)__insect / _signalRotationAngleR;
            int __atStep = (int)(__factor * SR);
            int __nextStep = __atStep + _torqueSteps;
            int __commutate = _debug[i+2];
            printf("lastAngleR: %d, angleR: %d, delta: %d, sector: %d, raw in sector: %d, factor: %f, atStep: %d, nextStep: %d, commutate: %d\n", __lastAngleR, __angleR, __delta, __sector, __insect, __factor, __atStep, __nextStep, __commutate);
        }
    }
}

// disengage motor
void Motor::disengage() {
    float totalPWMDuty = 100;                                           // motor is fully disengaged when 0 duty percentage for all coils (assume 100)
    int retries = 10;                                                   // doing retries is probably not needed here...
    while ((retries > 0) && (totalPWMDuty > 0)) {                       // try to disengage (could fail perhaps, to be investigated)
        retries--;                                                      // one retry
        mcpwm_set_duty(MCPWM_UNIT_0, _coil0, MCPWM_OPR_A, 0);           // disengage motor (duty cycle zero, no power)
        mcpwm_set_duty(MCPWM_UNIT_0, _coil1, MCPWM_OPR_A, 0);
        mcpwm_set_duty(MCPWM_UNIT_0, _coil2, MCPWM_OPR_A, 0);
        _running = false;                                               // assume disengage successful
        totalPWMDuty = mcpwm_get_duty(MCPWM_UNIT_0, _coil0, MCPWM_OPR_A) + mcpwm_get_duty(MCPWM_UNIT_0, _coil1, MCPWM_OPR_A) + mcpwm_get_duty(MCPWM_UNIT_0, _coil2, MCPWM_OPR_A); // what is current total duty
    }
    assert(totalPWMDuty == 0);                                          // this should not happen, can overheat coils and controller
}

// recalibrate
void Motor::recal() {
    determineSignalRotationAngleR();
}

// run test
// this test function can contain various code to tackle problems while testing
// current code is purely an example
void Motor::test() {
    int goal;
    int goalMod;
    int angle;
    float currentSignalFraction;
    int measuredStep = 0;
    int diff;
    home();
    for (int i = 0; i < _moveSteps; i++) {
        goal = measuredStep + _torqueSteps;
        goalMod = goal % SR;
        commutate(goal);
        ets_delay_us(SST);
        angle = _rps.getAngleR();
        currentSignalFraction = (float)(angle % _signalRotationAngleR) / _signalRotationAngleR;
        measuredStep = round(currentSignalFraction * SR);
        diff = measuredStep - (goalMod > SR ? goalMod-SR : goalMod);
        printf("step %d: goal-step was %d (%d), angle is %d, corresponds to step %d, diff = %d\n", i, goal, goalMod, angle, measuredStep, diff);
    }
    disengage();
}

// turn on debugRun (enables dump of run motor for debugging purposes)
void Motor::debugRun() {
    _debugRun = true;                                                   // enable debugRun
}

// set motor direction, use True for clockwise / False for counter clockwise 
void Motor::setDirection(bool clockwise) {
    if ((clockwise && !_clockwise) || (!clockwise && _clockwise)) {     // if current direction wrong
        reverseMotor();                                                 // reverse motor to match required direction
    }
}

// set motor direction reversed
// onlyPoles: use true to correct motor direction without reversing sensor
void Motor::reverseMotor(bool onlyPoles) {
    mcpwm_timer_t saveValue = _coil0;                                   // save coil0
    _coil0 = _coil1;                                                    // switch coil0 - coil1
    _coil1 = saveValue;
    if (!onlyPoles) {                                                   // if NOT only poles switch
        _clockwise = !_clockwise;                                       // remember state (register this state change)
        _rps.invertReadings();                                          // invert readings so we still get incremental (positive) readings
    }
}

// move motor to new position
// no steps parameter given: move will use _moveSteps property to make a multi commutate movement using the timer
// steps parameter given: move will use steps parameter to make small (one commutate) move within range of signal array without using the timer
void Motor::moveMotor(int steps) {
    if ((steps == 0) && (_moveSteps)) {                                 // if adhoc (parameter) value is 0 but moveSteps property has value
        _moveMode = true;                                               // set moveMode true (this will commutate using timer for bigger values, i.e. not bound to small stepsize within signal array range)
        _moveStepsLeft = _moveSteps;                                    // steps left (still to be made)
        _lastAngleR = _rps.getAngleR();
        startMotor();                                                   // start motor in this moveMode
        return;                                                         // return (don't do rest of logic for small adhoc stepping)
    }
    if (steps != 0) {                                                   // if we're still here and we have a non-zero steps parameter that means we should move a small (within signal array) step
        commutate(_lastStep + steps);                                   // so just commutate to that step
    }
}

// get current direction
int Motor::getDirection() {
    return _clockwise;                                                  // return direction CW = 1 / CCW = 0
}

// get current RPM
int Motor::getRPM() {
    return _stepFreq * 60 * ((float)_delta / _rpsResolution);           // return RPM based on average delta angle movement per step
}

// get motor angle in degrees
int Motor::getAngle() {
    _angleR = _rps.getAngleR();                                 // refresh last known angle
    return ((float)_angleR / _rpsResolution) * 360;             // return current motor angle (in degrees)
}

// get signal rotation angle in degrees
int Motor::getSignalRotationAngle() {
    return ((float)_signalRotationAngleR / _rpsResolution) * 360;   // return physical angle (in degrees) the motor moves on one full signal cycle
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
    _torqueSteps = angle * _SRFactor;                                   // calculate number of steps now to save time later
}

// set amplitude
void Motor::setAmplitude(int amplitude) {
    _amplitude = amplitude;                                             // set amplitude of the PWM as a percentage, this sets the power/current for the motor
}

// set move steps
void Motor::setMoveSteps(int steps) {
    _moveSteps = steps;                                                 // set amplitude of the PWM as a percentage, this sets the power/current for the motor
}

/*========================================================================*/
/*                            PRIVATE METHODS                             */
/*========================================================================*/

// setup pins
void Motor::setup_mcpwm_pins(motorConfig* MC) {
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MC->pin0A);                  // coil (a) PWM signal for driver to power motor coil
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MC->pin0B);                  // driver enable signal to enable half bridge for coil (a)
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, MC->pin1A);                  // coil (b) PWM signal for driver to power motor coil
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, MC->pin1B);                  // driver enable signal to enable half bridge for coil (b)
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, MC->pin2A);                  // coil (c) PWM signal for driver to power motor coil
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, MC->pin2B);                  // driver enable signal to enable half bridge for coil (c)
#ifdef NS_TIMER_DEBUG_PIN
        gpio_pad_select_gpio(NS_TIMER_DEBUG_PIN);                       // debug pin as a gpio function
        gpio_set_direction(NS_TIMER_DEBUG_PIN, GPIO_MODE_OUTPUT);       // set debug pin as output
#endif
}

// setup mcpwm configuration
void Motor::setup_mcpwm_configuration(int pwmFreq) {
    mcpwm_config_t pwm_config;                                          // mcpwm config parameter
    pwm_config.frequency = pwmFreq * 2;                                 // up-down counter uses two half cycles so for full cycle to have desired freq it has to be doubled
    pwm_config.cmpr_a = 50.0;                                           // duty cycle of PWMxA = 50.0% (temporary)
    pwm_config.cmpr_b = 50.0;                                           // duty cycle of PWMxB = 50.0% (temporary)
    pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER;                    // Up-down counter (triangle wave)
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
    mcpwm_set_duty(MCPWM_UNIT_0, _coil0, MCPWM_OPR_B, 100);             // set enable to continues High
    mcpwm_set_duty(MCPWM_UNIT_0, _coil1, MCPWM_OPR_A, 0);               // same for this coil
    mcpwm_set_duty(MCPWM_UNIT_0, _coil1, MCPWM_OPR_B, 100);             // same for this coil
    mcpwm_set_duty(MCPWM_UNIT_0, _coil2, MCPWM_OPR_A, 0);               // same for this coil
    mcpwm_set_duty(MCPWM_UNIT_0, _coil2, MCPWM_OPR_B, 100);             // same for this coil
    _running = false;                                                   // motor not running so running flag false
}

// fill svpwm lookup table
void Motor::setup_svpwm() {
    float factor = 115.4;                                               // amplitude factor (can be more that 100% due to space vector wobble effect)
    for (int i = 0; i < SR; i++) {                                      // one degree resolution should be sufficient but can be controlled with SR (Signal Resolution)
        float angle = ((float)i / SR) * 360;                            // calculate angle for array index
        float a = factor*sin(RAD(angle));                               // amplitude coil a
        float b = factor*sin(RAD(angle-120));                           // amplitude coil b
        float c = factor*sin(RAD(angle+120));                           // amplitude coil c
        float g = (MIN(a,b,c)+MAX(a,b,c))/2;                            // saw tooth pattern
        float duty = ((a - g) / 2) + 50;                                // lift wave form to DC (positive)
        _svpwm[i] = duty;                                               // set lookup value for step index
    }
}

// high resolution timer setup and start
void Motor::createTimer() {
    const esp_timer_create_args_t periodic_timer_args = {               // prepare config with callback and this motor instance
        &Motor::onTimer,
        this,
        ESP_TIMER_TASK,
        "onTimer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &_periodic_timer));  // create and save handle
}

// start timer
void Motor::startTimer(uint64_t interval) {
    ESP_ERROR_CHECK(esp_timer_start_periodic(_periodic_timer, interval));       // start timer with interval
}

// stop timer
void Motor::stopTimer() {
    ESP_ERROR_CHECK(esp_timer_stop(_periodic_timer));                   // stop timer
}

// callback for high resolution timer
void Motor::onTimer(void *arg) {
    Motor* m = ((Motor*)arg);                                           // create pointer to motor object as this is a static method (callback can not be an instance method)
    if (!m->_running) return;                                           // needed for disengage conflict (when timer fires during disengage), see second check at commutation
#ifdef NS_TIMER_DEBUG_PIN
    gpio_set_level(NS_TIMER_DEBUG_PIN, 1);                              // for timing reasons within #ifdef, signal high for timing debugging with scope
#endif
    m->_lastAngleR = m->_angleR;                                        // save last angle
    m->_angleR = m->_rps.getAngleR();                                   // get new (current) angle
    int lastDelta = m->_lastAngleR > m->_angleR ? m->_angleR - m->_lastAngleR + m->_rpsResolution :  m->_angleR - m->_lastAngleR;   // raw angle delta movement since last commutate request (adjusted for zero crossing)
    lastDelta = lastDelta > (m->_rpsResolution / 2) ? 0 : lastDelta;    // adjusted for wrong direction
    m->_delta = (m->_delta + lastDelta) / 2;                            // running average delta angle movement per step
#ifdef _NS_DYNAMIC_DEBUG
    if (m->_noProgressCount > 10) {                                     // for timing reasons within #ifdef, if longer time no progress
        m->_debugRun = true;                                            // dynamical switch debugRun
        m->_di = 0;                                                     // start at beginning of debug result queue
    }
    if (m->_delta < 90) {                                               // if movement smaller then some arbitrary value
        m->_noProgressCount++;                                          // increment noProgress counter for this event (gets higher if this condition repeats)
    } else {                                                            // else (move was within limits)
        m->_noProgressCount = m->_noProgressCount <= 0 ? 0 : m->_noProgressCount - 1;   // decrement noProgress to lower the noProgress level
    }
#endif
    float currentSignalFraction = (float)(m->_angleR % m->_signalRotationAngleR) / m->_signalRotationAngleR;    // take modulus to find position within signal cycle (same as using angle fraction * coils)
    int goal = (int)((currentSignalFraction * SR) + m->_torqueSteps);   // calculate goal step based on current measured position and requested torque
    if (m->_running && !m->_debugRun) {                                 // if running and not debugging
        if (m->_moveMode) {                                             // and mode is move
            if (m->_moveStepsLeft > m->_torqueSteps) {                  // steps left still more than one torque angle
                m->_moveStepsLeft -= m->_torqueSteps;                   // assume we move one torque angle
                m->commutate(goal);                                     // do the move
            } else {
                m->_moveMode = false;                                   // else end move mode
                m->stopMotor();                                         // and stop the motor
            }
        } else {
            m->commutate(goal);                                         // commutate immediately and leave here
        }
    } else if (m->_running) {                                           // if running and debugging do debugging stuff (takes more time so be careful with too long code)
        // *** example debug code ***
        m->_debug[m->_di] = m->_lastAngleR;                             // save last angle to debug queue
        m->_di++;                                                       // increment debug queue pointer
        m->_debug[m->_di] = m->_angleR;                                 // save current angle to debug queue
        m->_di++;                                                       // increment debug queue pointer
        m->_debug[m->_di] = goal;                                       // save goal to debug queue
        m->_di++;                                                       // increment debug queue pointer
        if (m->_di > 196) {                                             // stop debugging before queue end
            m->stopMotor();                                             // so stop motor (which will trigger dump of debug queue)
        } else {                                                        // if not at end of debug queue
            m->commutate(goal);                                         // perform commutate with current goal
        }
    }
#ifdef NS_TIMER_DEBUG_PIN
    gpio_set_level(NS_TIMER_DEBUG_PIN, 0);                              // signal low for debugging
#endif
}

// home motor; commutate to 0-position and reset angle
void Motor::home() {
    commutate(0);                                                       // commutate to 0-position
    ets_delay_us(SST);                                                  // let it settle
    _rps.resetAngleZero();                                              // reset angle
    _angleR = _rps.getAngleR();                                         // reset angle property
    _lastAngleR = _angleR;                                              // reset last angle property
}

// set motor at signal step
void Motor::commutate(int stepRequest) {
    int step = stepRequest >= SR ? stepRequest - SR : (stepRequest < 0 ? SR + stepRequest : stepRequest);   // adjust to signal array boundary
    if ((step >= SR) || step < 0) return;                                                                   // leave if that didn't work (request out of range)
    mcpwm_set_duty(MCPWM_UNIT_0, _coil0, MCPWM_OPR_A, ((float)_amplitude / 100) * _svpwm[step]);            // set signal values
    mcpwm_set_duty(MCPWM_UNIT_0, _coil1, MCPWM_OPR_A, ((float)_amplitude / 100) * _svpwm[step >= _phaseShift ? step - _phaseShift : step + _dblPhaseShift]);
    mcpwm_set_duty(MCPWM_UNIT_0, _coil2, MCPWM_OPR_A, ((float)_amplitude / 100) * _svpwm[step < _dblPhaseShift ? step + _phaseShift : step - _dblPhaseShift]);
    _lastStep = step;                                                                                       // remember last step
}

// do full physical turn and measure/calculate/save signal rotation angle (physical angle made in one signal cycle). Also correct direction when needed.
void Motor::determineSignalRotationAngleR() {
    int startAngle = 0;                                                 // temporary variables;
    int endAngle = 0;                                                   //
    bool negativeStart = false;                                         // signals negative (before zero crossing) position condition at start of probing
    assert(!_running);                                                  // motor should not be running
    home();                                                             // home motor and reset angle
    startAngle = _rps.getAngleR();                                      // get actual start angle
    if (startAngle > (_rpsResolution / 2)) negativeStart = true;        // position somewhere from 6-12 o'clock is considered negative
    for (int i = 0; i < _phaseShift; i++) { // 1/3 signal turn          // probe motor direction by (micro) stepping 1/3 turn in positive direction
        commutate(_lastStep + 1);                                       // move positive
        ets_delay_us(SST);                                              // settle
    }
    endAngle = _rps.getAngleR();                                        // get actual end angle
    if ((negativeStart && !(endAngle <= (_rpsResolution / 2))) ||       // if started negative and not landed in positive area
        (!negativeStart && (endAngle > (_rpsResolution / 2)))) {        // or not started negative but landed still in negative area
            reverseMotor(true);                                         // wrong direction (not CW) so reverse motor without inverting sensor
    }
    home();                                                             // home motor and reset angle
    startAngle = _rps.getAngleR();                                      // get actual start angle after homing
    if (startAngle > (_rpsResolution / 2)) {                            // if in negative area, look for zero crossing (into positive area)
        for (int i = 0; i < SR; i++) {                                  // microstep in positive direction (max full cycle)
            commutate(_lastStep + 1);                                   // move positive
            ets_delay_us(SST);                                          // settle
            startAngle = _rps.getAngleR();                              // set position as start angle
            if (startAngle < (_rpsResolution / 2)) {                    // if now in positive area, keep this start angle and leave
                break;                                                  // leave for next calibration step
            }                                                           // else try again to find zero crossing
        }
    }
    assert(startAngle < (_rpsResolution / 2));                          // if (after trying) zero crossing still not found; error state -> assert
    int quarterCount = 0;                                               // initialize temp var for count of quarters moved
    int delta = 0;                                                      // initialize temp var for delta of movement
    int minimalMove = 0;                                                // initialize temp var for finding smallest movement
    while (true) {                                                      // loop (exit condition --> in loop code below)
        commutate(_lastStep + _quarter);                                // move quarter positive
        quarterCount++;                                                 // count the move
        ets_delay_us(SST);                                              // settle
        endAngle = _rps.getAngleR();                                    // get actual angle after quarter move
        if ((quarterCount < 3) && (endAngle > (_rpsResolution / 2))) {  // still posible to get negative move at start of calibration loop
            quarterCount--;                                             // if that happens don't count move and retry
            continue;                                                   // start loop over (shortcut rest of code)
        }
        delta = endAngle - startAngle;                                  // calculate delta
        if ((minimalMove == 0) || (delta < minimalMove)) minimalMove = delta;   // save smallest value
        if ((endAngle < startAngle) || ((_rpsResolution - endAngle) < minimalMove)) break; // if last move end-position smaller then start we crossed zero crossing (or if a next move will no longer fit) full 360 turn done
        startAngle = endAngle;                                          // else prepare for next move (start angle = last end angle)
    }
    assert(quarterCount > 2);                                           // assume error condition when too few quarters moved
    _signalRotationAngleR = (_rpsResolution / quarterCount) * 4;        // else calculate one full signal rotation angle using number of quarters moved for full turn
    _clockwise = true;                                                  // motor now running clockwise (observed from front) do not set this state value directly after here, use setDirection() or reverseMotor()
    disengage();                                                        // disengage coils
}
