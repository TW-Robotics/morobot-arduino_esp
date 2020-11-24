#include <morobot.h>
#include <Dabble.h>

#define NUM_SERVOS 3

void setup() {
	Dabble.begin(9600);
	morobot.begin(NUM_SERVOS);
	
	delay(500);
	morobot.releaseBreaks();
	Serial.println("Bring the robot into zero position and press start.");
	Serial.println("Move the first two axes with your hands and use the gamepad to drive the linar axis completely in.");
}

void loop() {
	Dabble.processInput();
	
	if(GamePad.isPressed(0)) {			// UP
		morobot.moveAngle(3, -5, 1);
	} else if(GamePad.isPressed(1)) {	// DOWN
		morobot.moveAngle(3, 5, 1);
	} else if(GamePad.isPressed(4)) {	// START
		morobot.setZero();
		Serial.println("Axes set zero!");
	}
	
}