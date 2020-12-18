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
	gripper.begin();
	morobot.moveHome();
	delay(500);
	morobot.releaseBreaks();
	morobot.setSpeedRPM(20);
	
	/*Serial.println("Waiting for Dabble to connect to smartphone. If you are already connected, press any app-key.");
	Dabble.waitForAppConnection();
	Serial.println("Dabble connected!");*/
	
}

void loop() {
	Dabble.processInput();
	
	/*Serial.println(gripper.getCurrentOpeningAngle());
	gripper.morobot->smartServos.move(4, -5, 10);
	delay(100);
	Serial.println(gripper.getCurrentOpeningAngle());*/
	gripper.autoCalibrate();
	/*gripper.moveToAngle(-200, 10);
	gripper.moveToAngle(-50, 10);
	gripper.moveToAngle(-200, 10);
	gripper.moveToAngle(-50, 10);*/
	Serial.println(gripper.getCurrentOpeningAngle());
	delay(100000);
	
	
	/*if(GamePad.isPressed(0)) {			// Up
		morobot.moveAngle(2, -20, 30);
	} else if(GamePad.isPressed(1)) {   // Down
		morobot.moveAngle(2, 20, 30);
	} else if(GamePad.isPressed(2)) {	// Left

		delay(delayDebounce);
	} else if(GamePad.isPressed(3)) {   // Right

	} else if(GamePad.isPressed(7)) {	// O
		morobot.moveHome();
	} else if(GamePad.isPressed(8)) {   // X
		morobot.releaseBreaks();
	} else if(GamePad.isPressed(4)) {	// Start

	} else if(GamePad.isPressed(5)) {	// Select

		delay(delayDebounce);	
	} else if(GamePad.isPressed(6)) {	//Triangle
		Serial.println(morobot.getCurrent(1));
		Serial.println(morobot.getVoltage(1));
		Serial.println(morobot.getTemp(1));
		Serial.println(morobot.getSpeed(1));
		delay(delayDebounce);
	} else if(GamePad.isPressed(9)) {   // Square
		Serial.println("START COPYING HERE");
		
		Serial.println("STOP COPYING HERE");
		Serial.println("Just paste the code into the loop of the base_importantFunctionCalls-example (or make a new file with it).");
	}*/
}


