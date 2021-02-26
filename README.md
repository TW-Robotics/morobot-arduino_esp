[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

# morobot software library for microcontrollers such as Arduino and ESP32

This library can be used to control the UAS Technikum Wien morobots using an Arduino or ESP32 microcontroller.<br>
Visit https://tw-robotics.github.io/morobot-arduino_esp/ to read the documentation.

![](https://github.com/TW-Robotics/morobot-arduino_esp/blob/main/robot_info/morobot_types.png)

## Installation
Download this library and install it in your Arduino IDE. See the [Arduino Help Page](https://www.arduino.cc/en/guide/libraries#toc4) for more information.

## Usage
1. **Connect your robot**: Connect your morobot to the microcontroller and the power supply as shown in the picture below. Note the order in which you have to connect the motors (see the [robot info](https://github.com/TW-Robotics/morobot-arduino_esp/blob/main/robot_info/calibration-pose_coordinate-frames/))! The library is tested on Arduino Mega and ESP32 boards.
![](https://github.com/TW-Robotics/morobot-arduino_esp/blob/main/robot_info/morobot_connection.png)
2. **Calibrate your robot**: Run the example [calibrate_robot](https://github.com/TW-Robotics/morobot-arduino_esp/blob/main/examples/calibrate_robot/calibrate_robot.ino). You have to change the 'MOROBOT_TYPE' to match your robot model. Move the joints of the robot into their calibration position, which is shown on the graphics in the see the [robot info folder](https://github.com/TW-Robotics/morobot-arduino_esp/blob/main/robot_info/calibration-pose_coordinate-frames/). This ensures that in the future the joints can only move within a permitted range, which prevents the robot from being damaged. In addition, the kinematics can only be calculated correctly for calibrated robots.
3. **Have fun**: See the other [examples](https://github.com/TW-Robotics/morobot-arduino_esp/blob/main/examples/) to learn how to move the robot around and teach positions. You can use the robots e.g. in combination with the Dabble-App and Bluetooth-Module to control them with your smartphone.
4. **Using endeffectors**: To use a smartservo-gripper, just connect it to the last robot-servo. To use a normal servo just connect it to the microcontroller and see the [endeffector-example](https://github.com/TW-Robotics/morobot-arduino_esp/blob/main/examples/endeffector/endeffector.ino) on using it.

### Examples
- base_importantFunctionCalls<br>
  Bare minimum program and function calls for controlling a morobot-s rrp. Just put control commands into the loop and look what happens.
- calibrate_robot<br>
  Change the origin position of all motors of the robot. These values are stored in the motors so this program must only be called once to initially calibrate the robot. Check the robot-info for details on the zero-positions of the motors.
- driveAround<br>
  Control all morobots with the dabble app and drive around giving angular values or x-y-z-coordinates.
- multiple_robots<br>
  Control multiple robots (in this case 2x morobot-s) with the dabble app. Use the app to switch between the robots and drive the axes of the robots directly.
- teach_robot<br>
  Move the robot around and store positions using the Dabble-App. The robot can than drive to these positions autonomously. You can also export all positions as movement comments.
- endeffector<br>
  Use the different grippers in combination with the robots.
### Supported microcontrollers
- Arduino Mega - Control up to 3 morobots
- ESP32 - Control up to 2 morobots
### Supported robot types
- morobot-s (rrp)
- morobot-s (rrr)
- morobot-2d
- morobot-3d
- morobot-p
### Supported grippers
- Smart-Servo Parallel-Gripper
- Micro-Servo Angular-Gripper
- Binary Endeffectors (Pumps, Electromagnets, Motors, ...) - Connected directly or via relais

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
- TCP-Offsets in y-directions not implemented for morobot-p and morobot-s (rrp and rrr)

## License
This software is licensed under the terms of the GNU General Public License v3.0. See the [LICENSE](https://github.com/TW-Robotics/morobot-arduino_esp/blob/main/LICENSE) for more information.
