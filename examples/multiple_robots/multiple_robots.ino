#include <morobot.h>
#include <Dabble.h>

#define NUM_SERVOS 3

int delayDebounce = 500;

morobotClass morobot1;
morobotClass morobot2;
morobotClass* actMorobot;

void setup() {
	Dabble.begin(9600);
	morobot1.begin(NUM_SERVOS, "Serial2");
	morobot2.begin(NUM_SERVOS, "Serial1");
	morobot1.moveHome();
	morobot2.moveHome();
	delay(500);
	morobot1.setSpeedRPM(25);
	morobot2.setSpeedRPM(25);
	actMorobot = &morobot1;
}

void loop() {
	//TODO: Check if connected to dabble
	Dabble.processInput();

	if(GamePad.isPressed(2)) {			// Left
		if (actMorobot->getActAngle(0) <= 100) actMorobot->moveAngle(0, 3, 1);
	} else if(GamePad.isPressed(3)) {   // Right
		if (actMorobot->getActAngle(0) >= -100) actMorobot->moveAngle(0, -3, 1);
	} else if(GamePad.isPressed(0)) {	// Up
		if (actMorobot->getActAngle(1) <= 100) actMorobot->moveAngle(1, 3, 1);
	} else if(GamePad.isPressed(1)) {   // Down
		if (actMorobot->getActAngle(1) >= -100) actMorobot->moveAngle(1, -3, 1);	
	} else if(GamePad.isPressed(6)) {	//Triangle
		if (actMorobot->getActAngle(2) > 0) actMorobot->moveAngle(2, -10, 20);
	} else if(GamePad.isPressed(8)) {   // X
		if (actMorobot->getActAngle(2) < 780) actMorobot->moveAngle(2, 10, 20);
	} else if(GamePad.isPressed(7)) {	// O
		Serial.println(actMorobot->getActAngle(0));
		Serial.println(actMorobot->getActAngle(1));
		Serial.println(actMorobot->getActAngle(2));
		delay(delayDebounce);
	} else if(GamePad.isPressed(5)) {	// Select
		if (actMorobot == &morobot1) actMorobot = &morobot2;
		else actMorobot = &morobot1;
		delay(delayDebounce);	
	}
}
