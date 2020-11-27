/**
 *  \file base_importantFunctionCalls.ino
 *  \brief Bare minimum program and function calls for controlling a morobot Scara RRP. Just put control commands into the loop and look what happens.
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

#include <morobotScaraRRP.h>  	// If you are using another robot, change the name to the correct header file here

morobotScaraRRP morobot;		// And change the class-name here

void setup() {
	morobot.begin("Serial2");
}

void loop() {
	// Functions to define parameters
	morobot.setSpeedRPM(5);				// Set the global speed for all motors here. This value can be overwritten temporarily if a function is called with a speed parameter explicitely.
	morobot.setTCPoffset(10.2, -3.4, 9.2);	// If there is an endeffector connected to the last axis give its position with respect to the center of the flange here
	
	// Functions to get the robot state
	if (morobot.checkIfAngleValid(0, 45)) Serial.println("Motor 0 can drive to 45 degrees");
	else Serial.println("Unable to reach 45 degrees with motor 0");
	if (morobot.checkIfAngleValid(1, 105)) Serial.println("Motor 1 can drive to 105 degrees");
	else Serial.println("Unable to reach 105 degrees with motor 1");
	delay(5000);

	Serial.println("Parameters of motor 2:");
	Serial.print("Temperature [degrees Celsius]: ");
	Serial.println(morobot.getTemp(2));
	Serial.print("Speed [RPM]: ");
	Serial.println(morobot.getSpeed(2));
	Serial.print("Voltage [Volt]: ");
	Serial.println(morobot.getVoltage(2));
	Serial.print("Current [Ampere]: ");
	Serial.println(morobot.getCurrent(2));
	Serial.print("Angle [degree]: ");
	Serial.println(morobot.getActAngle(2));
	Serial.print("Y-Position of TCP [mm]: ");
	Serial.println(morobot.getActPosition('y'));
	delay(5000);
	
	// Manipulating the breaks
	Serial.println("Now you can't move the robot with your hands");
	morobot.setBreaks();
	delay(5000);
	
	Serial.println("Now you can move the robot with your hands");
	morobot.releaseBreaks();
	delay(5000);	
	
	// Moving the robot around
	// with absolute values
	morobot.moveHome();	
	morobot.moveToAngle(0, 60);			// Move motor 0 to 60 degrees
	morobot.waitUntilIsReady();			// Wait until robot is there before sending next movement
	morobot.moveToAngle(0, -60, 1);		// Move motor 0 to -60 degrees with smallest speed
	while(1) if (morobot.checkIfMotorMoves(0) == false) break;		// Wait until motor 0 stops moving
	delay(5000);

	long angles[3] = {37, -59, 260};
	morobot.moveToAngles(angles);		// Or you can define an array and send it to the robot
	angles[1] = 59;
	morobot.moveToAngles(angles, 50);	// You can also send additionally a desired speed in RPM
	delay(5000);
	
	//with relative values
	morobot.moveHome();	
	morobot.moveAngle(1, 95);			// Move motor 1 by 95 degrees
	morobot.waitUntilIsReady();			// Wait until robot is there before sending next movement
	morobot.moveAngle(1, -60, 10);		// Move motor 1 to -60 degrees with small speed
	while(1) if (morobot.checkIfMotorMoves(0) == false) break;		// Wait until motor 1 stops moving
	delay(5000);
	
	long angles1[3] = {-37, -5, 260};	// Move motor 0 by -37 degrees, motor 1 by -5 degrees, ...
	morobot.moveAngles(angles1);			// You can define an array and send it to the robot. The robot waits until all motors are finished autonomously at all functions giving multiple angles.
	angles1[0] = 37;
	morobot.moveAngles(angles1, 50);		// You can also send additionally a desired speed in RPM
	morobot.printAngles(angles);		// Print angle-array to Serial monitor
	delay(5000);
	
	// in a coordinate system (all functions wait until robot stops moving before beginning)
	morobot.moveToPosition(160.0, 45.3, 28.1);		// Move the robot to an x-y-z-position. Function returns False if the position is not reachable
	morobot.moveXYZ(7, -10.1, -13.2);				// Move the robot relatively along the x-y-z-axes. Function returns False if the position is not reachable
	morobot.moveInDirection('x', 7);				// Move the robot relatively along one axis. Function returns False if the position is not reachable
	delay(15000);
}
