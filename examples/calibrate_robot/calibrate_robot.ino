/**
 *  \file calibrate_robot.ino
 *  \brief Change the origin position of all motors of the robot. These values are stored in the motors so this program must only be called once to initially calibrate the robot.
 *  @author	Johannes Rauer FHTW
 *  @date	2020/11/27
 *  
 *  Hardware: 		- Arduino Mega (or similar microcontroller)
 *  				- HC-05 bluetooth controller (or similar)
 *  				- Calibrated morobot
 *  				- Powersupply 9-12V 5A (or more)
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

// **********************************************************************
// ********************* CHANGE THIS LINES ******************************
// **********************************************************************
#define MOROBOT_TYPE 	morobot_3d		// morobot_s_rrr, morobot_s_rrp, morobot_2d, morobot_3d, morobot_p
#define USE_DABBLE		0				// Necessary for all robots with linear axes! 0 - don't use dabble app; 1 - use dabble app

#include <morobot_s_rrr.h>
#include <morobot_s_rrp.h>
#include <morobot_2d.h>
#include <morobot_3d.h>
#include <morobot_p.h>

#if USE_DABBLE != 0
	#ifndef ESP32
	#include <Dabble.h>			// Include Dabble library for AVR-based controllers (Arduino) if no ESP32 is used
	#define DABBLE_PARAM 9600	// Set transmission speed
	#else
	#include <DabbleESP32.h>	// Include Dabble library for ESP32 board
	#define DABBLE_PARAM "MyEsp32" // Set bluetooth name
	#endif
#endif

MOROBOT_TYPE morobot;			// And change the class-name here
bool currentLimitReached = false;

void setup() {
	morobot.begin("Serial2");	
	delay(500);
	morobot.releaseBreaks();
	
	#if USE_DABBLE != 0
		Dabble.begin(DABBLE_PARAM);	
		Serial.println("Waiting for Dabble to connect to smartphone. If you are already connected, press any app-key.");
		Dabble.waitForAppConnection();
		Serial.println("Dabble connected!");
		Serial.println("Bring the robot into zero position and press the start-button.");
		Serial.println("Use the gamepad (up/down) to drive the linear axis. Move the other axes with your hands to zero position.");
	#else
		Serial.println("Bring the robot into zero position and wait. If you miss the moment, just reset the microcontroller.");
		Serial.println("All motors are set zero in 5 seconds");
		delay(5000);
		morobot.setZero();
		Serial.println("Axes set zero!");
	#endif
}

void loop() {
	#if USE_DABBLE != 0
		Dabble.processInput();
		
		if (morobot.type == "morobot_s_rrp") {
			if (GamePad.isPressed(0) && currentLimitReached == false) {			// UP
				morobot.moveAngle(2, -2, 1, false);
			} else if(GamePad.isPressed(1)) {									// DOWN
				morobot.moveAngle(2, 2, 1, false);
				currentLimitReached = false;
			}
			if (morobot.getCurrent(2) > 25) {
				Serial.println("Motor stopped due to current limit being reached");
				currentLimitReached = true;
			}
		}
		
		if(GamePad.isPressed(4)) {											// START
			morobot.setZero();
			Serial.println("Axes set zero!");
			delay(500);
		}
	#endif
}