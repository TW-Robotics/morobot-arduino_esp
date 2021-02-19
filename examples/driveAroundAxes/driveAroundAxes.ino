/**
 *  \file driveAroundAxes.ino
 *  \brief Control the robot with the dabble app and drive around giving axes angles
 *  @author	Johannes Rauer FHTW
 *  @date	2021/02/17
 *  
 *  Hardware: 		- Arduino Mega (or similar microcontroller)
 *  				- HC-05 bluetooth controller (or similar)
 *  				- Calibrated morobot
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

#include <morobot_p.h>
#ifndef ESP32
#include <Dabble.h>			// Include Dabble library for AVR-based controllers (Arduino) if no ESP32 is used
#define DABBLE_PARAM 9600	// Set transmission speed
#else
#include <DabbleESP32.h>	// Include Dabble library for ESP32 board
#define DABBLE_PARAM "MyEsp32" // Set bluetooth name
#endif

// Create morobot object and declare variables
morobot_p morobot;
float step = 3.0;
int delayDebounce = 0;

void setup() {
	Dabble.begin(DABBLE_PARAM);		// Start connection to Dabble
	morobot.begin("Serial2");		// The robot is connected to RX/TX2 -> Serial2
	morobot.setSpeedRPM(50);
	//morobot.setTCPoffset(4.92,34.1,10); // If the robot has an endeffector set its position here (e.g. Pen-Holder)
	//morobot.moveHome();				// Move the robot into initial position
	//morobot.moveZAxisIn();			// Move the z-axis in (in case it is out more than one rotation of motor on startup)
	//morobot.setZero();				// Set the axes zero when move in
	delay(200);
	
	Serial.println("Waiting for Dabble to connect to smartphone. If you are already connected, press any app-key.");
	Dabble.waitForAppConnection();
	Serial.println("Dabble connected!");
	Serial.println("Use the gamepad to drive the robot around. Triangle and X buttons control vertical axis.");
}

void loop() {
	// Get the input from the app
	Dabble.processInput();
	
	if(GamePad.isPressed(2)) {			// Left
		morobot.moveAngle(0, step);
		delay(delayDebounce);
	} else if(GamePad.isPressed(3)) {	// Right
		morobot.moveAngle(0, -step);
		delay(delayDebounce);
	} else if(GamePad.isPressed(0)) {	// Up
		morobot.moveAngle(1, step);
		delay(delayDebounce);
	} else if(GamePad.isPressed(1)) {	// Down
		morobot.moveAngle(1, -step);
		delay(delayDebounce);
	} else if(GamePad.isPressed(6)) {	// Triangle
		morobot.moveAngle(2, step);
		delay(delayDebounce);
	} else if(GamePad.isPressed(8)) {   // X
		morobot.moveAngle(2, -step);
		delay(delayDebounce);
	} else if(GamePad.isPressed(7)) {	// O
		morobot.moveHome();
	} else if(GamePad.isPressed(9)) {   // Square
		morobot.printTCPpose();
		Serial.println(morobot.getActAngle(0));
		Serial.println(morobot.getActAngle(1));
		Serial.println(morobot.getActAngle(2));
	}
}
