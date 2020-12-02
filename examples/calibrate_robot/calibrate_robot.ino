/**
 *  \file calibrate_robot.ino
 *  \brief Change the origin position of all motors of the robot. These values are stored in the motors so this program must only be called once to initially calibrate the robot.
 *  @author	Johannes Rauer FHTW
 *  @date	2020/11/27
 *  
 *  Hardware: 		- Arduino Mega (or similar microcontroller)
 *  				- HC-05 bluetooth controller (or similar)
 *  				- morobot RRP
 *  				- Powersupply 12V 5A (or more)
 *  Connections:	- Powersupply to Arduino hollow connector
 *  				- First smart servo of robot to Arduino:
 *  					- Red cable to Vin
 *  					- Black cable to GND
 *  					- Yellow cable to pin 16 (TX2)
 *  					- White calbe to pin 17 (RX2)
 *  				- HC-05 to Arduino (Configuration to work with Dabble):
 *  					- 5V to 5V
 *  					- GND to GND
 *  					- RX to pin 14 (TX3)
 *  					- TX to pin 15 (RX3)
 *  Install the Dabble-App on your smartphone or tablet
 */

#include <morobotScaraRRP.h>  	// If you are using another robot, change the name to the correct header file here
#ifndef ESP32
#include <Dabble.h>			// Include Dabble library for AVR-based controllers (Arduino) if no ESP32 is used
#define DABBLE_PARAM 9600	// Set transmission speed
#else
#include <DabbleESP32.h>	// Include Dabble library for ESP32 board
#define DABBLE_PARAM "MyEsp32" // Set bluetooth name
#endif

morobotScaraRRP morobot;			// And change the class-name here
bool currentLimitReached = false;

void setup() {
	Dabble.begin(DABBLE_PARAM);
	morobot.begin("Serial2");
	
	delay(500);
	morobot.releaseBreaks();
	
	Serial.println("Waiting for Dabble to connect to smartphone. If you are already connected, press any app-key.");
	Dabble.waitForAppConnection();
	Serial.println("Dabble connected!");
	Serial.println("Bring the robot into zero position and press the start-button.");
	Serial.println("Move the first two axes with your hands and use the gamepad to drive the linar axis completely in.");
}

void loop() {
	Dabble.processInput();
	
	if(GamePad.isPressed(0) && currentLimitReached == false) {			// UP
		morobot.moveAngle(2, -2, 1, false);
	} else if(GamePad.isPressed(1)) {	// DOWN
		morobot.moveAngle(2, 2, 1, false);
		currentLimitReached = false;
	} else if(GamePad.isPressed(4)) {	// START
		morobot.setZero();
		Serial.println("Axes set zero!");
	}
	if (morobot.getCurrent(2) > 25) {
		Serial.println("Motor stopped due to current limit being reached");
		currentLimitReached = true;
	}
}