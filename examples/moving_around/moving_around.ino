/**
 *  \file base_importantFunctionCalls.ino
 *  \brief Bare minimum program and function calls for controlling a morobot-s (rrp). Just put control commands into the loop and look what happens.
 *  @author	Johannes Rauer FHTW
 *  @date	2020/11/27
 *  
 *  Hardware: 		- Arduino Mega (or similar microcontroller)
 *  				- morobot RRP
 *  				- Powersupply 12V 5A (or more)
 *  Connections:	- Powersupply to Arduino hollow connector
 *  				- First smart servo of robot to Arduino:
 *  					- Red cable to Vin
 *  					- Black cable to GND
 *  					- Yellow cable to pin 16 (TX2)
 *  					- White calbe to pin 17 (RX2)
 */

#include <morobot_p.h>  	// If you are using another robot, change the name to the correct header file here

morobot_p morobot;		// And change the class-name here

void setup() {
	morobot.begin("Serial2");
	morobot.moveHome();				// Move the robot into initial position
	morobot.setSpeedRPM(2);				// Set the global speed for all motors here. This value can be overwritten temporarily if a function is called with a speed parameter explicitely.
}

void loop() {
	morobot.moveToPosition(-80, 80, 30);
	morobot.moveInDirection('z', -25);
	delay(1000);
	morobot.moveInDirection('z', 25);
	delay(500);
	morobot.moveToAngle(0, 0);
	morobot.moveToAngle(1 , 75);
	morobot.moveToAngle(2, -25);
	delay(500);
	morobot.moveToPosition(160, -80, 100);
	morobot.setSpeedRPM(2);
	morobot.waitUntilIsReady();
	morobot.moveXYZ(60, -60, 0);
	morobot.waitUntilIsReady();
	//morobot.moveInDirection('z', -25);
	delay(2000);
	//morobot.moveInDirection('z', 25);
	morobot.waitUntilIsReady();
	morobot.moveXYZ(-60, 60, 0);
	morobot.waitUntilIsReady();
	morobot.setSpeedRPM(2);
	delay(2000);
	morobot.moveToAngle(0, 0);
	morobot.moveToAngle(1 , 75);
	morobot.moveToAngle(2, -25);
	delay(5000);
	
}
