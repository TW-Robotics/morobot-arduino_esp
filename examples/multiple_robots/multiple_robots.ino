/**
 *  \file multiple_robots.ino
 *  \brief Control multiple robots (in this case 2x morobot-s (rrp)) with the dabble app. Use the app to switch between the robots and drive the axes of the robots directly.
 *  @author	Johannes Rauer FHTW
 *  @date	2020/11/27
 *  
 *  Hardware: 		- Arduino Mega (or similar microcontroller)
 *  				- HC-05 bluetooth controller (or similar)
 *  				- 2x calibrated morobot RRP
 *  				- Powersupply 9-12V 5A (or more)
 *  Connections:	- Powersupply to Arduino hollow connector
 *  				- First smart servo of robot 1 to Arduino:
 *  					- Red cable to Vin
 *  					- Black cable to GND
 *  					- Yellow cable to pin 16 (TX2) (Serial2)
 *  					- White calbe to pin 17 (RX2)
  *  				- First smart servo of robot 2 to Arduino:
 *  					- Red cable to Vin
 *  					- Black cable to GND
 *  					- Yellow cable to pin 18 (TX1) (Serial1)
 *  					- White calbe to pin 19 (RX1)
 *  				- HC-05 to Arduino (Configuration to work with Dabble):
 *  					- 5V to 5V
 *  					- GND to GND
 *  					- RX to pin 14 (TX3)
 *  					- TX to pin 15 (RX3)
 *  Install the Dabble-App on your smartphone or tablet
 */

#include <morobot_s_rrr.h>
#include <morobot_s_rrp.h>
#include <morobot_2d.h>
#include <morobot_3d.h>
#include <morobot_p.h>

#ifndef ESP32
#include <Dabble.h>			// Include Dabble library for AVR-based controllers (Arduino) if no ESP32 is used
#define DABBLE_PARAM 9600	// Set transmission speed
#else
#include <DabbleESP32.h>	// Include Dabble library for ESP32 board
#define DABBLE_PARAM "MyEsp32" // Set bluetooth name
#endif

int delayDebounce = 250;
int angleStep = 6;
int angleVertStep = 20;

// **********************************************************************
// *************** CHANGE THE ROBOT TYPES HERE **************************
// **********************************************************************
morobot_3d morobot1;
morobot_p morobot2;

morobotClass* actMorobot;	// Pointer to store which robot is active at the moment

void setup() {
	Dabble.begin(DABBLE_PARAM);
	// **********************************************************************
	// *************** CHANGE THE SERIAL PORTS HERE *************************
	// **********************************************************************
	morobot1.begin("Serial1");
	morobot2.begin("Serial2");
	delay(500);
	morobot1.moveHome();
	morobot2.moveHome();
	delay(500);
	morobot1.setSpeedRPM(25);
	morobot2.setSpeedRPM(25);
	actMorobot = &morobot1;		// Start with morobot 1
	
	Serial.println("Waiting for Dabble to connect to smartphone. If you are already connected, press any app-key.");
	Dabble.waitForAppConnection();
	Serial.println("Dabble connected!");
	Serial.println("Use the gamepad to drive the robot around. Triangle and X buttons control vertical axis. O button drives robot into initial position. SELECT button switches between the robots");
}

void loop() {
	Dabble.processInput();

	if(GamePad.isPressed(2)) {			// Left
		actMorobot->moveAngle(0, angleStep);
	} else if(GamePad.isPressed(3)) {   // Right
		actMorobot->moveAngle(0, -angleStep);
	} else if(GamePad.isPressed(0)) {	// Up
		actMorobot->moveAngle(1, angleStep);
	} else if(GamePad.isPressed(1)) {   // Down
		actMorobot->moveAngle(1, -angleStep);	
	} else if(GamePad.isPressed(6)) {	// Triangle
		actMorobot->moveAngle(2, -angleVertStep);
	} else if(GamePad.isPressed(8)) {   // X
		actMorobot->moveAngle(2, angleVertStep);
	} else if(GamePad.isPressed(7)) {	// O
		Serial.println(actMorobot->getActAngle(0));
		Serial.println(actMorobot->getActAngle(1));
		Serial.println(actMorobot->getActAngle(2));
		actMorobot->moveHome();
		delay(delayDebounce);
	} else if(GamePad.isPressed(5)) {	// Select
		if (actMorobot == &morobot1) actMorobot = &morobot2;	// Switch to other robot
		else actMorobot = &morobot1;
		delay(delayDebounce);	
	}
}
