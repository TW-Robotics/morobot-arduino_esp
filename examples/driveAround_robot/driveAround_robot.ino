#include <morobot.h>
#include <Dabble.h>

#define NUM_SERVOS 3

morobotClass morobot;
float step = 0.25;
int delayDebounce = 250;
float actPos[3];
float actPosTemp[3];

void setup() {
	Dabble.begin(9600);
	morobot.begin(NUM_SERVOS, "Serial2");
	morobot.setSpeedRPM(50);
	morobot.setTCPpos(4.92,-34.1,10);
	initVars();
	morobot.moveHome();	
}

void loop() {
	//TODO: Check if connected to dabble	
	Dabble.processInput();
	
	if(GamePad.isPressed(2)) {
		actPosTemp[1] = actPos[1] - step;
	} else if(GamePad.isPressed(3)) {
		actPosTemp[1] = actPos[1] + step;
	} else if(GamePad.isPressed(0)) {
		actPosTemp[0] = actPos[0] + step;
	} else if(GamePad.isPressed(1)) {
		actPosTemp[0] = actPos[0] - step;
	} else if(GamePad.isPressed(6)) {	//Triangle
		actPosTemp[2] = actPos[2] - step;
	} else if(GamePad.isPressed(8)) {   // X
		actPosTemp[2] = actPos[2] + step;
	} else if(GamePad.isPressed(7)) {	// O
		morobot.moveHome();
		initVars();
	} else if(GamePad.isPressed(9)) {   // Square
		morobot.releaseBreaks();
	}
	
	if (actPosTemp[0] != actPos[0] || actPosTemp[1] != actPos[1] || actPosTemp[2] != actPos[2]){
		if (morobot.moveToPosition(actPosTemp[0], actPosTemp[1], actPosTemp[2]) == false){
			delay(delayDebounce);
		} else {
			for (uint8_t i=0; i<3; i++) actPos[i] = actPosTemp[i];
		}
	}
}

void initVars(){
	actPos[0] = morobot.getActPosition('x');
	actPos[1] = morobot.getActPosition('y');
	actPos[2] = morobot.getActPosition('z');
	for (uint8_t i=0; i<3; i++) actPosTemp[i] = actPos[i];
}
