#include <morobot.h>
#include <Dabble.h>

#define MAX_NUM_POS 20
#define NUM_SERVOS 3

long motorAngles[NUM_SERVOS][MAX_NUM_POS];
int delayTime = 1500;
int delayDebounce = 500;
int posId = 0;
int idxPlayback = -1;

morobotClass morobot;

void setup() {
	Dabble.begin(9600);
	morobot.begin(NUM_SERVOS, "Serial1");
	morobot.moveHome();	
	delay(500);
	morobot.releaseBreaks();
	Serial.println("'Select' to store position, 'Start' to start moving, 'O' to move to init position, 'Left'/'Right' to jump through positions stored, 'Up'/'Down' to move last axis.");
	morobot.setSpeedRPM(25);
}

void loop() {
	//TODO: Check if connected to dabble
	Dabble.processInput();
	
	if(GamePad.isPressed(0)) {			// Up
		if (morobot.getActAngle(2) > 0) morobot.moveAngle(2, -5, 1);
	} else if(GamePad.isPressed(1)) {   // Down
		if (morobot.getActAngle(2) < 780) morobot.moveAngle(2, 5, 1);
	} else if(GamePad.isPressed(2)) {	// Left
		if (idxPlayback == -1){
			Serial.println("Store positions before going back");
		} else {
			long angles[3] = {motorAngles[0][idxPlayback], motorAngles[1][idxPlayback], motorAngles[2][idxPlayback]};
			morobot.moveToAngles(angles);
			delay(delayTime);
			idxPlayback--;
			if (idxPlayback == -1) idxPlayback = posId - 1;	// if last position is reached
		}
		delay(delayDebounce);
	} else if(GamePad.isPressed(3)) {   // Right
		if (idxPlayback == -1){
			Serial.println("Store positions before going through them");
		} else {
			long angles[3] = {motorAngles[0][idxPlayback], motorAngles[1][idxPlayback], motorAngles[2][idxPlayback]};
			morobot.moveToAngles(angles);
			delay(delayTime);
			idxPlayback++;
			if (idxPlayback == posId) idxPlayback = 0;	// if last position is reached
		}
		delay(delayDebounce);
	} else if(GamePad.isPressed(7)) {	// O
		morobot.moveHome();
	} else if(GamePad.isPressed(8)) {   // X
		morobot.releaseBreaks();
	} else if(GamePad.isPressed(4)) {	// Start
		for (int i=0; i<posId; i++) {
			long angles[3] = {motorAngles[0][i], motorAngles[1][i], motorAngles[2][i]};
			morobot.moveToAngles(angles);
			delay(delayTime);
		}
	} else if(GamePad.isPressed(5)) {	// Select
		if (posId < MAX_NUM_POS){
			for (byte i=0; i<NUM_SERVOS; i++) motorAngles[i][posId] = morobot.getActAngle(i);
			Serial.print("Stored position ");
			Serial.print(posId);
			Serial.print(": ");
			printAnglesFromIdx(posId);
			idxPlayback = posId;
			posId++;
		} else {
			Serial.println("Maximum number ob positions reached");
		}
		delay(delayDebounce);	
	} else if(GamePad.isPressed(6)) {	//Triangle
		Serial.println(morobot.getCurrent(1));
		Serial.println(morobot.getVoltage(1));
		Serial.println(morobot.getTemp(1));
		Serial.println(morobot.getSpeed(1));
		delay(delayDebounce);
	}
}

void printAnglesFromIdx(long idx){
	long ang[NUM_SERVOS];
	for (byte i=0; i<NUM_SERVOS; i++) ang[i] = motorAngles[i][idx];
	morobot.printAngles(ang);
}
