[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

# morobot software library for microcontrollers such as Arduino and ESP32

This library can be used to control the UAS Technikum Wien morobots using an Arduino or ESP32 microcontroller.<br>
Visit https://tw-robotics.github.io/morobot-arduino_esp/ to read the documentation,

## Installation
Download this library and install it in your Arduino IDE. See the [Arduino Help Page](https://www.arduino.cc/en/guide/libraries#toc4) for more information.

## Usage
See the examples on how to use the library.
### Examples
- base_importantFunctionCalls<br>
  Bare minimum program and function calls for controlling a morobot-s rrp. Just put control commands into the loop and look what happens.
- calibrate_robot<br>
  Change the origin position of all motors of the robot. These values are stored in the motors so this program must only be called once to initially calibrate the robot.
- driveAroundAxes<br>
  Control all morobot with the dabble app and drive around giving angular values
- driveAroundXYZ_morobot-s(rrp)<br>
  Control the morobot-s (rrp) with the dabble app and drive around giving x-y-z-coordinates
- driveAroundXYZ_morobot-s(rrp)<br>
  Control the morobot-s (rrr) with the dabble app and drive around giving x-y-z-coordinates
- driveAroundXYZ_morobot-2d<br>
  Control the morobot-2d with the dabble app and drive around giving x-y-z-coordinates
- multiple_robots<br>
  Control multiple robots (in this case 2x morobot-s) with the dabble app. Use the app to switch between the robots and drive the axes of the robots directly.
- teach_robot<br>
  Move the robot around and store positions using the Dabble-App. The robot can than drive to these positions autonomously. You can also export all positions as movement comments.
- endeffector<br>
  Use the different grippers in combination with the robots.
### Supported robot types
- morobot-s (rrp)
- morobot-s (rrr)
- morobot-2d
### Supported grippers
- Smart-Servo Parallel-Gripper
- Micro-Servo Angular-Gripper

## How to add a new robot
The class morobotClass is an abstract base class implementing functions like moving the robot's joints. For each new robot type, a new child class with corresponding header- and cpp-file has to be implemented. Start with newRobotClass_Template and make the following changes:
- Copy the newRobotClass_Template files and rename them. Include type and kinematics into name
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

## Known issues
- Example with multiple robots is not working for ESP32 controllers
- Other TCP-Offset than x/0/x for morobot-s (rrp and rrr) is not implemented (necessary?)
- Other EEF-types (e.g. pumps with relais) not implemented yet

## License
This software is licensed under the terms of the GNU General Public License v3.0. See the [LICENSE](https://github.com/TW-Robotics/morobot/edit/main/LICENSE) for more information.
