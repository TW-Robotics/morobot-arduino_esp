/**
 *  \file endeffecotr.ino
 *  \brief TODO
 *  @author	Johannes Rauer FHTW
 *  @date	2020/12/18
 *  
 *  Hardware: 		- Arduino Mega (or similar microcontroller)
 *  				- HC-05 bluetooth controller (or similar)
 *  				- morobot RRR (or similar) with endeffector
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

#include <morobotScaraRRR.h>  	// If you are using another robot, change the name to the correct header file here
#ifndef ESP32
#include <Dabble.h>			// Include Dabble library for AVR-based controllers (Arduino) if no ESP32 is used
#define DABBLE_PARAM 9600	// Set transmission speed
#else
#include <DabbleESP32.h>	// Include Dabble library for ESP32 board
#define DABBLE_PARAM "MyEsp32" // Set bluetooth name
#endif

#include <eef.h>

#define MAX_NUM_POS 20
#define NUM_SERVOS 3

long motorAngles[NUM_SERVOS][MAX_NUM_POS];
int delayTime = 1500;
int delayDebounce = 500;
int posId = 0;
int idxPlayback = -1;

morobotScaraRRR morobot;
gripper gripper(&morobot);

void setup() {
	Dabble.begin(DABBLE_PARAM);
	morobot.begin("Serial2");
	gripper.begin(9);
	gripper.setSpeed(50, 25);
	
	morobot.moveHome();
	delay(500);
	morobot.releaseBreaks();
	morobot.setSpeedRPM(20);
	
	Serial.println("Waiting for Dabble to connect to smartphone. If you are already connected, press any app-key.");
	Dabble.waitForAppConnection();
	Serial.println("Dabble connected!");
	
}

void loop() {
	Dabble.processInput();
	
	if(GamePad.isPressed(0)) {			// Up
		gripper.moveAngle(-10);		// Open gripper
		delay(delayDebounce/5);
	} else if(GamePad.isPressed(1)) {   // Down
		gripper.moveAngle(10);		// Close gripper
		delay(delayDebounce/5);
	} else if(GamePad.isPressed(2)) {	// Left
		Serial.println(gripper.getCurrentOpeningAngle());	// Display opening angle and width
		//Serial.println(gripper.getCurrentOpeningWidth());
		delay(delayDebounce);
	} else if(GamePad.isPressed(3)) {	// Right
		gripper.autoCalibrate();	// Calibrate
	} else if(GamePad.isPressed(7)) {	// O
		gripper.open();
	} else if(GamePad.isPressed(8)) {   // X
		gripper.close();
	} else if(GamePad.isPressed(4)) {	// Start
		gripper.closeToForce();
	} else if(GamePad.isPressed(5)) {	// Select
		gripper.moveToAngle(90, 10);
	} 
}


