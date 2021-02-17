/**
 *  \file endeffecotr.ino
 *  \brief TODO
 *  @author	Johannes Rauer FHTW
 *  @date	2020/12/22
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

#include <morobot_s_rrr.h>  	// If you are using another robot, change the name to the correct header file here
#ifndef ESP32
#include <Dabble.h>			// Include Dabble library for AVR-based controllers (Arduino) if no ESP32 is used
#define DABBLE_PARAM 9600	// Set transmission speed
#else
#include <DabbleESP32.h>	// Include Dabble library for ESP32 board
#define DABBLE_PARAM "MyEsp32" // Set bluetooth name
#endif

#include <eef.h>

int delayDebounce = 100;

morobot_s_rrr morobot;
gripper gripper(&morobot);

void setup() {
	Dabble.begin(DABBLE_PARAM);
	morobot.begin("Serial2");
	//gripper.begin(9);			// For normal servo gripper
	gripper.begin();			// For smart servo gripper
	gripper.setSpeed(50, 25);	// Set speed for opening and closing the gripper
	//gripper.setParams(65, 140, 50, 155, 1.0, 0);	// Call this function with the correct parameters only if you are using a non-standard gripper
													// Parameters: degClosed, degOpen, degCloseLimit, degOpenLimit, gearRatio, openWidthOffset; values for servomotor -> degClosed is defined as 0 degrees in real angles. 
	//setTCPoffset(0, 0, -18);	// Call this function with the correct parameters only if you are using a non-standard gripper
	
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
	
	if(GamePad.isPressed(0)) {			// Up - Open gripper a little bit
		gripper.moveWidth(5);			// For smart servo control width or angle
		//gripper.moveAngle(10);		// For normal servo control angle
		delay(delayDebounce);
	} else if(GamePad.isPressed(1)) {   // Down - Close gripper a little bit
		gripper.moveWidth(-5);			// For smart servo control width or angle
		//gripper.moveAngle(-10);		// For normal servo control angle
		delay(delayDebounce);
	} else if(GamePad.isPressed(2)) {	// Left
		Serial.println(gripper.getCurrentOpeningAngle());	// Display opening angle and width
		Serial.println(gripper.getCurrentOpeningWidth());
		delay(delayDebounce*5);
	} else if(GamePad.isPressed(3)) {	// Right
		gripper.autoCalibrate();		// Calibrate (Only for smart-servo gripper)
	} else if(GamePad.isPressed(7)) {	// O
		gripper.open();
	} else if(GamePad.isPressed(8)) {   // X
		gripper.close();
	} else if(GamePad.isPressed(4)) {	// Start
		gripper.closeToForce();
		delay(delayDebounce);
	} else if(GamePad.isPressed(5)) {	// Select
		gripper.moveToAngle(90, 10);
		gripper.moveToAngle(-15, 1);
		gripper.moveToAngle(20, 15);
		gripper.moveToAngle(90);
		gripper.moveToWidth(105);		// Only for smart-servo gripper
		gripper.moveToWidth(150, 15);	// Only for smart-servo gripper
		gripper.closeToForce();			// Only for smart-servo gripper
	} 
}


