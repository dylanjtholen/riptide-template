# RIPTide Template

This project is a PROS V5 template developed by 8110Z with:

-   a starter arcade drive setup
-   a chassis helper class for autonomous motion
-   a simple autonomous selector using the controller screen
-   support for a motor group, IMU, and optional odometry-based movement

## Requirements

Before using this template, make sure you have:

-   PROS installed and configured
-   a VEX V5 brain connected
-   the PROS extension or CLI available in your development environment

## Quick Start

1. Clone the repo and open the folder in VS Code.
2. Review the starter code in src/main.cpp.
3. Set the hardware ports and robot constants.
4. Build and upload the program to the V5 brain.

## Configure Your Robot

The main file already contains placeholder values that you must replace:

-   motor group ports in src/main.cpp
-   IMU port in src/main.cpp
-   robot dimensions and drive constants near the top of the file

Important values to tune:

-   wheelDiam
-   trackWidth
-   gearRatio
-   cartRPM

These values affect the accuracy of autonomous movement and driver control.

## Driver Control

The default opcontrol() function uses arcade drive:

-   left joystick Y controls forward and backward motion
-   right joystick X controls turning

If you want, you can also replace the starter arcade drive with tank drive, split arcade, or a custom control scheme.

You can change this behavior in the opcontrol() function inside src/main.cpp.

## Autonomous Programming

Autonomous code is organized in the functions below:

-   auto0()
-   auto1()
-   auto2()
-   auto3()
-   auto4()
-   auto5()

The autonomous() function selects which one runs based on the current auton selection.

## Autonomous Selector

The template includes a simple selector that uses the V5 screen and controller buttons:

-   the LCD buttons allow you to cycle through autonomous options
-   the currently selected option is displayed on the brain screen

You can customize the available autons and their labels in src/main.cpp.

## Project Structure

-   src/main.cpp: main robot code and operator/autonomous logic
-   src/chassis.cpp: chassis motion control implementation
-   include/chassis.h: chassis class declaration
-   include/main.h: standard PROS include header
