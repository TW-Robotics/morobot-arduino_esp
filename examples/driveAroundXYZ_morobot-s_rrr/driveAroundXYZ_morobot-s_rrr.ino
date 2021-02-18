/**
 *  \file driveAroundXYZ_morobot-s(rrr).ino
 *  \brief Control the morobot-s(rrr) with the dabble app and drive around giving x-y-z-coordinates
 *  @author	Johannes Rauer FHTW
 *  @date	2020/12/17
 *  
 *  Hardware: 		- Arduino Mega (or similar microcontroller)
 *  				- HC-05 bluetooth controller (or similar)
 *  				- Calibrated morobot-s (rrr)
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

#include <morobot_s_rrr.h>
#include <eef.h>
#ifndef ESP32
#include <Dabble.h>			// Include Dabble library for AVR-based controllers (Arduino) if no ESP32 is used
#define DABBLE_PARAM 9600	// Set transmission speed
#else
#include <DabbleESP32.h>	// Include Dabble library for ESP32 board
#define DABBLE_PARAM "MyEsp32" // Set bluetooth name
#endif

// Create morobot object and declare variables
morobot_s_rrr morobot;
gripper gripper(&morobot);
float step = 1.0;
int delayDebounce = 250;
float actPos[3];
float actPosTemp[3];

void setup() {
	Dabble.begin(DABBLE_PARAM);		// Start connection to Dabble
	morobot.begin("Serial2");		// The robot is connected to RX/TX2 -> Serial2
	morobot.setSpeedRPM(50);
	morobot.setTCPoffset(70, 0, 10); // If the robot has an endeffector set its position here (y-coordinate not supported)
	morobot.moveHome();				// Move the robot into initial position
	morobot.setZero();				// Set the axes zero when move in
	gripper.begin();			// For smart servo gripper
	gripper.setSpeed(50, 25);	// Set speed for opening and closing the gripper
	delay(200);
	initVars();
	
	Serial.println("Waiting for Dabble to connect to smartphone. If you are already connected, press any app-key.");
	Dabble.waitForAppConnection();
	Serial.println("Dabble connected!");
	Serial.println("Use the gamepad to drive the robot around. Triangle and X buttons control last axis.");
}

void loop() {
	// Get the input from the app
	Dabble.processInput();
	
	if(GamePad.isPressed(2)) {			// Left
		actPosTemp[1] = actPos[1] - step;
	} else if(GamePad.isPressed(3)) {	// Right
		actPosTemp[1] = actPos[1] + step;
	} else if(GamePad.isPressed(0)) {	// Up
		actPosTemp[0] = actPos[0] - step;
	} else if(GamePad.isPressed(1)) {	// Down
		actPosTemp[0] = actPos[0] + step;
	} else if(GamePad.isPressed(6)) {	// Triangle
		actPosTemp[2] = actPos[2] + step;
	} else if(GamePad.isPressed(8)) {   // X
		actPosTemp[2] = actPos[2] - step;
	} else if(GamePad.isPressed(7)) {	// O
		morobot.moveHome();
		initVars();
	} else if(GamePad.isPressed(9)) {   // Square
		morobot.getActPosition('x');
		initVars();
	} else if(GamePad.isPressed(4)) {	// Start
		gripper.open();
	} else if(GamePad.isPressed(5)) {   // Select
		gripper.close();
	}
	
	// If a movement comment has been given, move the robot
	if (actPosTemp[0] != actPos[0] || actPosTemp[1] != actPos[1] || actPosTemp[2] != actPos[2]){
		if (morobot.moveToPosition(actPosTemp[0], actPosTemp[1], actPosTemp[2]) == false){
			delay(delayDebounce);
			for (uint8_t i=0; i<3; i++) actPosTemp[i] = actPos[i];
		} else {
			for (uint8_t i=0; i<3; i++) actPos[i] = actPosTemp[i];
		}
	}
}

void initVars(){
	actPos[0] = morobot.getActPosition('x');
	actPos[1] = morobot.getActPosition('y');
	actPos[2] = morobot.getActOrientation('z');
	for (uint8_t i=0; i<3; i++) actPosTemp[i] = actPos[i];
}