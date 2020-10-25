SwarmDrive BLDC Example
=======================

This repo contains a (first) well documented example intended to be used for the SwarmDrive motor driver board.

SwarmDrive is a motor driver (development) board intended for learning and experimenting with electric motors. It's an approachable mechatronics platform for users who want to learn and start experimenting with (small) electric motors and BLDC motors in particular. The board contains a basic motor driver setup together with a powerful microcontroller (ESP32) and USB connectivity. It enables the user to learn about different commutation algorithms and all other aspects of electric (BLDC) motors in a convenient way. The _Swarm_ aspect is hinting upon the vast communication possibilities of the ESP32 such as bluetooth or wifi which enables the board or motor to communicate with other boards/motors. This allows for Swarm implementations or just plain remote control of the board/motors.

Read more about it on: [swarmdrive.nickstick.nl](https://swarmdrive.nickstick.nl)

[![SwarmDrive logo](https://swarmdrive.nickstick.nl/assets/img/swarmdrive_logo_blue.png)](https://swarmdrive.nickstick.nl)
[![SwarmDrive board](https://swarmdrive.nickstick.nl/assets/img/swarmdrive_image.png)](https://swarmdrive.nickstick.nl)

This example
------------

This example drives a BLDC motor, it's by no means a perfect implementation but just serves as a starter example. It consists of a Motor Class and makes use of a console library for operating the motor or changing its parameters. This console is an in-firmware implementation which means that it runs on the SwarmDrive on-board MCU and communicates with a serial monitor. So any serial monitor (even trough bluetooth or wifi) get access to the same console screen to operate the motor.

Also built-in is a small I2C implementation for communicating with a Rotary Encoder, as this example is meant to be a closed loop implementation for driving the motor. It needs feedback from a sensor. The sensor used for this example is the low cost AS5048B from AMS. The example was tested using a cheap Gimbal (BLDC) motor (AX-2206 80T 11.3 Ohm) which can be found for under $10.

The code was written using the ESP-IDF framework from Espressif because it uses the on-board motor control unit(s). But it can be ported to Arduino in a mixed ESP-IDF/Arduino setup. PlatformIO would be a suitable tool to work with.

The console
-----------

When running this example in GUI mode a screen will appear showing possible commands and parameters that can be changed. A log section show messages of what's going on and a command line for entering commands. The commands that are shown at the top section like "Run motor (r)" can be entered (**r** in this case) or parameters shown (like "Direction (d)") can be adjusted by typing d=1 for clockwise or d=0 for counter clockwise. Arrow up/dwn can be used for recalling commands.

Of course a rotary encoder like the AS5048B needs to be connected and attached to the motor. A potentiometer for controlling the motor speed is optional. As this example is a very crude implementation you have to watch the temperature of the motor and the driver. There are no provisions for it in the code.

When using the monitor functionality of PlatformIO it can be necessary to put the following line in platformio.ini:

monitor_filters = direct

This enables the special terminal characters used by the console running in GUI mode. Or you can run the console in NON-GUI mode (which is also the best way when using some sort of remote console).

On Windows the typical console window is dumb and does not support any escapes. When ANSI.sys is loaded it supports some escapes. Also here you could use NON-Gui mode or fix the handling of escape codes.

General Remarks
---------------

During the time of this writing (in the now current ESP-IDF version) some driver defenitions used for the MCPWM module are missing. See https://github.com/espressif/esp-idf/issues/5429 for now these have to be added in MCPWM.h which, most likely, can be found at (homedir)/.platformio/packages/framework-espidf/components/driver/include/driver.

** EDIT: I noticed that the type definition moved to (homedir)/.platformio/packages/framework-espidf/components/soc/include/hal/mcpwm_types.h. So for newer versions of esp-idf look for it there.

Original ===>

```
/**
 * @brief MCPWM select sync signal input
 */
typedef enum {
    MCPWM_SELECT_SYNC0 = 4,  /*!<Select SYNC0 as input*/
    MCPWM_SELECT_SYNC1,      /*!<Select SYNC1 as input*/
    MCPWM_SELECT_SYNC2,      /*!<Select SYNC2 as input*/
} mcpwm_sync_signal_t;
```

Changed ===>

```
/**
 * @brief MCPWM select sync signal input
 */
typedef enum {
    MCPWM_SELECT_TIMER0 = 1, // PWM timer0 sync_out
    MCPWM_SELECT_TIMER1,     // PWM timer1 sync_out
    MCPWM_SELECT_TIMER2,     // PWM timer2 sync_out
    MCPWM_SELECT_SYNC0,      /*!<Select SYNC0 as input*/
    MCPWM_SELECT_SYNC1,      /*!<Select SYNC1 as input*/
    MCPWM_SELECT_SYNC2,      /*!<Select SYNC2 as input*/
} mcpwm_sync_signal_t;
```

vTaskList undefined
-------------------

The code uses vTaskList, it has to be enabled in the sdkconfig (or you can comment out the code in ns_console.cpp). To enable
vTaskList use "platformio run -t menuconfig" (if you're using platformio) to enable it. Before using this command
make sure your screen is big enough or else you will get errors. Then enable:

```
Component config --->
FreeRTOS --->
[*] Enable FreeRTOS trace facility
[*]   Enable FreeRTOS stats formatting functions (NEW)

exit/save
```

mbedTLS error
-------------

If you get an error: “Please configure IDF framework to include mbedTLS -> Enable pre-shared-key ciphersuites and activate at least one cipher” you need to enable these using menuconfig:

platformio run -t menuconfig (make sure your screen is not too small, or this won’t run)

```
Component Config -> mbedTLS -> TLS Key Exchange Methods -> 
      [*] Enable pre-shared-key ciphersuits
        [*] Enable PSK based ciphersuite modes
```

When more information is available on the release of the SwarmDrive Development Board it will be posted here.
