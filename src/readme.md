SwarmDrive BLDC Example
=======================

This repo contains a (first) well documented example intended to be used for the SwarmDrive motor driver board. SwarmDrive is currently in development and this repo will be updated when it will be released.

SwarmDrive is a motor driver (development) board intended for learning and experimenting with electric motors. It's an approachable mechatronics platform for users who want to learn and start experimenting with (small) electric motors and BLDC motors in particular. The board contains a basic motor driver setup together with a powerful microcontroller (ESP32) and USB connectivity. It enables the user to learn about different commutation algorithms and all other aspects of electric (BLDC) motors in a convenient way. The _Swarm_ aspect is hinting upon the vast communication possibilities of the ESP32 such as bluetooth or wifi which enables the board or motor to communicate with other boards/motors. This allows for Swarm implementations or just plain remote control of the board/motors.

More information will follow...

This example
------------

This example drives a BLDC motor, it's by no means a perfect implementation but just serves as a starter example. It consists of a Motor Class and makes use of a console library for operating the motor or changing its parameters. This console is an in-firmware implementation which means that it runs on the SwarmDrive on-board MCU and communicates with a serial monitor. So any serial monitor (even trough bluetooth or wifi) get access to the same console screen to operate the motor.

Also built-in is a small I2C implementation for communicating with a Rotary Encoder, as this example is meant to be a closed loop implementation for driving the motor. It needs feedback from a sensor. The sensor used for this example is the low cost AS5048B from AMS. The example was tested using a cheap Gimbal (BLDC) motor (AX-2206 80T 11.3 Ohm) which can be found for under $10.

The code was written using the ESP-IDF framework from Espressif because it uses the on-board motor control unit(s). But it can be ported to Arduino in a mixed ESP-IDF/Arduino setup. Also there are other Arduino based libraries available. PlatformIO would be a suitable tool to work with but for simple code experiments the Arduino IDE will do great as well.

The console
-----------

When running this example in GUI mode a screen will appear showing possible commands and parameters that can be changed. A log section show messages of what's going on and a command line for entering commands. The commands that are shown at the top section like "Run motor (r)" can be entered (**r** in this case) or parameters shown (like "Direction (d)") can be adjusted by typing d=1 for clockwise or d=0 for counter clockwise. Arrow up/dwn can be used for recalling commands.

Of course a rotary encoder like the AS5048B needs to be connected and attached to the motor. A potentiometer for controlling the motor speed is optional. As this example is a very crude implementation you have to watch the temperature of the motor and the driver. There are no provisions for it in the code.

When using the monitor functionality of PlatformIO it can be necessary to put the following line in platformio.ini:

monitor_filters = direct

This enables the special terminal characters used by the console running in GUI mode. Or you can run the console in NON-GUI mode (which is also the best way when using some sort of remote console).

General Remarks
---------------

During the time of this writing (in the now current ESP-IDF version) some driver defenitions used for the MCPWM module are missing. See https://github.com/espressif/esp-idf/issues/5429 for now these have to added in MCPWM.h

When more information is available on the release of the SwarmDrive Development Board it will be posted here.




