# ARDUINO OMNIBOT 9 - Test of kinematic model
> Arduino sketch which uses a kinematic model to convert an desired robot speed into the correct motor speeds, and the measured motor speeds into an estimated robot speed and pose, for an omnidirectional moving base with three wheels.

## Table of contents
* [General info](#general-info)
* [List of files](#list-of-files)
* [Screenshots](#screenshots)
* [Setup](#setup)
* [Status](#status)
* [Inspiration](#inspiration)
* [Contact](#contact)

## General info
Arduino sketch which uses a kinematic model to convert an desired robot speed into the correct motor speeds, and the measured motor speeds into an estimated robot speed and pose, for an omnidirectional moving base with three wheels. The encoder signals are read using digital inputs and interruptions. The interruptions are configured by accessing directly the ATMEGA328 registers, which makes the sketch compatible only with Arduino UNO. This can be changed if the imterruption configuration for the encoder inputs is implemented using [EnableInterrupt](https://github.com/GreyGnome/EnableInterrupt) Arduino interrupt library, designed for a variety of Arduino boards. 
The sketch was prepared for a moving base with the following configurations:

* Three omnidirectional wheels with 25.0 cm radius
* Base with 87.5 cm radius
* Angle betwee wheel's rotation axis and base radius equal to zero degrees
* Pulse per revolution (PPR) of the motor-gearbox-encoder setup equal to 937 PPR
* Motor supply voltage equal to 12.0 V
* Maximum wheel speed is not used by the code
* Pinout definitions

´´´
#define SLPpin1           10        /* H-bridge enable pin 1 */
#define SLPpin2           11        /* H-bridge enable pin 2 */
#define Encoder1PinA      A0        /* Encoder input A for motor 1 */
#define Encoder1PinB      A1        /* Encoder input B for motor 1 */
#define Encoder2PinA      A2        /* Encoder input A for motor 2 */
#define Encoder2PinB      A3        /* Encoder input B for motor 2 */
#define Encoder3PinA      A4        /* Encoder input A for motor 3 */
#define Encoder3PinB      A5        /* Encoder input B for motor 3 */
#define Motor1Dir         7         /* Motor 1 direction output pin */
#define Motor1PWM         6         /* Motor 1 PWM output pin */
#define Motor2Dir         8         /* Motor 2 direction pin */
#define Motor2PWM         9         /* Motor 2 PWM output pin */
#define Motor3Dir         12        /* Motor 3 direction pin */
#define Motor3PWM         3         /* Motor 3 PWM output pin */
´´´

## List of files
* 9-test_kinematic_model.ino - Arduino sketch with main code
* dcmotor.cpp - Source file with dc motor drive using PWM
* dcmotor.h - Header file with class for dc motor drive using PWM
* myalgebra.c - Source file with functions for basic vector and matrix operations
* myalgebra.h - Header file with functions for basic vector and matrix operations
* pid.cpp - Source file with implementation of PID controller discretization using bilinear transform
* pid.h - Header file with class for PID controller
* README.md - Readme file
* rotary_encoder.cpp - Source file with methods to decode, count and read rotary encoders
* rotary_encoder.h - Source file with class for rotary encoders
* tracking_loop_filter.cpp - Source file with implementatio of tracking loop filter
* tracking_loop_filter.h - Header file with implementatio of tracking loop filter
* octave_serial_remote_controller_example/serial_remote_controller.m - Octave script to interact with the mobile base (robot)
* octave_serial_remote_controller_example/srl_fread.m - Function to enable reading data from serial port
* octave_serial_remote_controller_example/srl_fwrite.m - Function to enable writing data to serial port

## Screenshots
![Example screenshot](./img/screenshot.png)

## Setup

* First you need to install the Arduino IDE (if you haven't already) from [this link](https://www.arduino.cc/en/software).
* Download Octave from [this link](https://www.gnu.org/software/octave/download)
* Install the following Octave packages and their dependencies: control, mapping, instrument-control, optim, signal, communications. To install a package, for exemple, control, use the following command on the Octave's command window:
'''
pkg install -forge control
'''
A dependency, if any is indicated in an error message, can be installed using the same method
* Open "9-test_kinematic_model.ino" sketch with Arduino IDE and program the Arduino.
* Now, if you open the serial monitor, baud = 115200, you will see The Arduino printing motor speeds, motor speed setpoints, robot speeds, robot speed setpoints and robot pose. it is possible to change robot speed by sending a command in the format Sx;y;z;, where x is the desired robot speed on the x direction, y is the desired speed on the y direction, and z is the speed of rotation arround the robot's center. For example, send S0.1;0.0;0.0; to send the robot moving at 0.1 m/s in the x direction, S-0.1;0.0;0.0; to send the robot moving at 0.1 m/s in the negative x direction, S0.0;0.1;0.0; to send the robot moving at 0.1 m/s in the y direction, S0.0;0.0;10.0; to make the robot spin at 10 RPM (Rotaions per Minute). Any combination and value is possible, limited only by the maximum and minumum rotational speeds of the motors.
* Take note of the Arduino's serial port name, it should be used to configure the serial port on the Octave script.
* Open the Octave stript "octave_serial_remote_controller_example/serial_remote_controller.m" with Octave, change COMPORT variable to the name of serial port you just took note of (the example code is for a serial port on Linux", and run the script.
* If a error message appear on the screen, please verify if the Arduino is connected, if the COMPORT value is correct, and if you have the permissions to access the serial port.
* If a lot of numbers, representing robot setpoint and robot speed, appear on the command window, click type on the command window W, A, S and D (capital) to drive the robot arround, Q (capital) to stop it, and X (capital) to terminate the script execution. You can verify if the key press was detected by the values displayed on the command window.

The robot can be connected to the computer using any sort of link, e.g., USB cable, a pair of Zigbees, a bluetooth dongle such as HC-06.
A remote control a cellphone or a second Arduino + joystick (use your imagination) can be configured based on the things done in the Octave script and the Arduino sketch.


It is possible to customize the arduino code to change pin connections, wheel radius, moving base radius, number of wheels and their angles, and so forth, by reading the skech. I might update this readme file with a How-To (and screenshots), but it is not guaranteed to hapen.

## Status
Project is: _in progress_.

## Inspiration
The functions for operations with matrices and vectors were adapted from [MatrixMath for Arduino](https://github.com/eecharlie/MatrixMath)

The class and methods for rotary encoders were adapted from [RotaryEncoder for Arduino](https://github.com/mathertel/RotaryEncoder)

The template for this readme file was created by [@flynerdpl](https://www.flynerd.pl/).

## Contact
Created by [@passoswell](https://github.com/passoswell).
