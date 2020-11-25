#include <morobot.h>
#include <Dabble.h>

#define NUM_SERVOS 3

morobotClass morobot;
float step = 5;
int delayDebounce = 250;
float actPos[3];
float actPosTemp[3];
bool changed = false;

void setup() {
	Dabble.begin(9600);
	morobot.begin(NUM_SERVOS, "Serial2");
	morobot.moveHome();	
	morobot.setSpeedRPM(25);
	actPos[0] = morobot.getActPosition('x');
	actPos[1] = morobot.getActPosition('y');
	actPos[2] = morobot.getActPosition('z');
	for (uint8_t i=0; i<3; i++) actPosTemp[i] = actPos[i];
	//morobot.setTCPpos(20,-35,0);
}

void loop() {
	//TODO: Check if connected to dabble	
	Dabble.processInput();
	
	if(GamePad.isPressed(2)) {
		actPosTemp[1] = actPos[1] - step;
		changed = true;
	} else if(GamePad.isPressed(3)) {
		actPosTemp[1] = actPos[1] + step;
		changed = true;
	} else if(GamePad.isPressed(0)) {
		actPosTemp[0] = actPos[0] + step;
		changed = true;
	} else if(GamePad.isPressed(1)) {
		actPosTemp[0] = actPos[0] - step;
		changed = true;
	} else if(GamePad.isPressed(7)) {	// O
		morobot.moveHome();
	} else if(GamePad.isPressed(6)) {	//Triangle
		actPosTemp[2] = actPos[2] - step;
		changed = true;
	} else if(GamePad.isPressed(8)) {   // X
		actPosTemp[2] = actPos[2] + step;
		changed = true;
	} else if(GamePad.isPressed(9)) {   // Square
		morobot.releaseBreaks();
	}
	
	if (changed == true){
		if (morobot.moveToPosition(actPosTemp[0], actPosTemp[1], actPosTemp[2]) == false){
			delay(delayDebounce);
		} else {
			for (uint8_t i=0; i<3; i++) actPos[i] = actPosTemp[i];
		}
		changed = false;
	}
	
	/*if(GamePad.isPressed(2)) {
		morobot.moveInDirection('y', -step);
	} else if(GamePad.isPressed(3)) {
		morobot.moveInDirection('y', step);
	} else if(GamePad.isPressed(0)) {
		morobot.moveInDirection('x', step);
	} else if(GamePad.isPressed(1)) {
		morobot.moveInDirection('x', -step);
	} else if(GamePad.isPressed(7)) {	// O
		morobot.moveHome();
	} else if(GamePad.isPressed(6)) {	//Triangle
		morobot.moveInDirection('z', -step);
		delay(delayDebounce);	
	} else if(GamePad.isPressed(8)) {   // X
		morobot.moveInDirection('z', step);
		delay(delayDebounce);	
	} else if(GamePad.isPressed(9)) {   // Square
		morobot.releaseBreaks();
	}*/
}
