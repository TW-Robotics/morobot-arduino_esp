# morobot software library for microcontrollers such as Arduino and ESP32

This library can be used to control the UAS Technikum Wien morobots using an Arduino or ESP32 microcontroller.

## Installation
Download this library and install it in your Arduino IDE. See the [Arduino Help Page](https://www.arduino.cc/en/guide/libraries#toc4) for more information.

## Usage
See the examples on how to use the library.
### Examples
- calibrate_robot
- driveAround_robot
- multiple_robots
- teach_robot
### Supported robot types
- Scara RRP

## Add a new robot
The class morobotClass is an abstract base class implementing functions like moving the robot's joints. For each new robot type, a new child class with corresponding header- and cpp-file has to be implemented. Start with newRobotClass_Template and make the following changes:
- Copy the newRobotClass_Template files and rename them. Include type and kinematics into name (e.g. Scara RRP = morobotScaraRRP)
- Make the following changes in the **.h-file** (See TODOs in file):
  - Search for 'newRobotClass_Template' and replace all instances with your class name (same name as .h and .cpp file)
  - Change the number of servos of the robot in the call of the morobotClass constructor
  - Change the Number of servos and the joint limits in the variable '_jointLimits[][]'
  - Add variables necessary to solve the inverse kinematics (e.g. lengths of the axes)
- Make the following changes in the **.cpp-file** (See TODOs in file):
  - Search for 'newRobotClass_Template' and replace all instances with your class name (same name as .h and .cpp file)
  - In 'setTCPoffset()' add your code to calculate the new lengths of the last axis including the EEF-offsets
  - In 'calculateAngles()' implement the inverse kinematics of the robot
  - In 'updateCurrentXYZ' implement the forward kinematics of the robot
- To use your robot type, simple add an include with your header file name into your file and create an instance of the robot object (see examples for existing robots)