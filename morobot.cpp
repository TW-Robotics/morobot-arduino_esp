#include <Arduino.h>
#include <morobot.h>
#include <SoftwareSerial.h>
#include <MakeblockSmartServo.h>

//TODO: Calibrate with motor currents

morobotClass::morobotClass(uint8_t numSmartServos){
	if (numSmartServos > NUM_MAX_SERVOS){
		Serial.print("Too many motors! Maximum number of motors: ");
		Serial.println(NUM_MAX_SERVOS);
	}
	_numSmartServos = numSmartServos;
}

void morobotClass::begin(const char* stream){
	Serial.begin(115200);
	
	if (stream == "Serial1") { //https://www.arduino.cc/reference/en/language/functions/communication/serial/
		Serial1.begin(115200);
		_port = &Serial1;
	} else if (stream == "Serial2") {
		Serial2.begin(115200);
		_port = &Serial2;
	} else if (stream == "Serial3") {
		Serial3.begin(115200);
		_port = &Serial3;
	}
	else Serial.println("ERROR: Serial-Parameter not valid");
	
    smartServos.beginSerial(_port);
	//delay(5);
    smartServos.assignDevIdRequest();
    delay(50);
	
	setTCPoffset(0, 0, 0);

	Serial.println("Connected to motors");
}

void morobotClass::setZero(){
	for (uint8_t i=0; i<_numSmartServos; i++) smartServos.setZero(i+1);
}

void morobotClass::moveHome(){
	for (uint8_t i=0; i<_numSmartServos; i++) smartServos.setInitAngle(i+1);
}

void morobotClass::setSpeedRPM(uint8_t speed){
	_speedRPM = speed;
	if (speed > SERVO_MAX_SPEED_RPM) _speedRPM = SERVO_MAX_SPEED_RPM;
	if (speed < 0) _speedRPM = 1;
}


/* BREAKS */

void morobotClass::setBreaks(){
	for (uint8_t i=0; i<_numSmartServos; i++) smartServos.setBreak(i+1, BREAK_BRAKED);
}

void morobotClass::releaseBreaks(){
	for (uint8_t i=0; i<_numSmartServos; i++) smartServos.setBreak(i+1, BREAK_LOOSE);
}


/* ROBOT STATUS */

void morobotClass::setBusy(){
	for (uint8_t i=0; i<_numSmartServos; i++) _angleReached[i] = false;
}

void morobotClass::setIdle(){
	for (uint8_t i=0; i<_numSmartServos; i++) _angleReached[i] = true;
}

bool morobotClass::isReady(){
	for (uint8_t i=0; i<_numSmartServos; i++) {
		if (_angleReached[i] == false) {
			if (checkIfMotorMoves(i+1) == false) continue;
			return false;
		}
	}
	return true;
}

void morobotClass::waitUntilIsReady(){
	unsigned long startTime = millis();
	while (true){
		if (isReady() == true) break;
		if ((millis() - startTime) > TIMEOUT_DELAY) {
			Serial.println("Timeout");
			break;
		}
	}
	setIdle();
}

bool morobotClass::checkIfMotorMoves(uint8_t servoId){
	long startPos = getActAngle(servoId);
	delay(50);
	if (startPos != getActAngle(servoId)) return true;
	return false;
}


/* GETTERS */

long morobotClass::getActAngle(uint8_t servoId){
	return smartServos.getAngleRequest(servoId+1);
}

float morobotClass::getActPosition(char axis){
	updateCurrentXYZ();
	
	if (axis == 'x') return _actPos[0];
	else if (axis == 'y') return _actPos[1];
	else if (axis == 'z') return _actPos[2];
	else Serial.println("ERROR! Invalid axis in getActPosition();");
}

float morobotClass::getSpeed(uint8_t servoId){
	return smartServos.getSpeedRequest(servoId+1);
}

float morobotClass::getTemp(uint8_t servoId){
	return smartServos.getTempRequest(servoId+1);
}

float morobotClass::getVoltage(uint8_t servoId){
	return smartServos.getVoltageRequest(servoId+1);
}

float morobotClass::getCurrent(uint8_t servoId){
	return smartServos.getCurrentRequest(servoId+1);
}


/* MOVEMENTS */

void morobotClass::moveToAngle(uint8_t servoId, long angle){
	if (checkIfAngleValid(servoId, angle) == true) smartServos.moveTo(servoId+1, angle, _speedRPM);
}

void morobotClass::moveToAngle(uint8_t servoId, long angle, uint8_t speedRPM){
	if (checkIfAngleValid(servoId, angle) == true) smartServos.moveTo(servoId+1, angle, speedRPM);
}

void morobotClass::moveToAngles(long angles[]){
	waitUntilIsReady();
	setBusy();
	Serial.print("Moving to [deg]: ");
	printAngles(angles);
	
	for (uint8_t i=0; i<_numSmartServos; i++) moveToAngle(i, angles[i]);
}

void morobotClass::moveToAngles(long angles[], uint8_t speedRPM){
	waitUntilIsReady();
	setBusy();
	Serial.print("Moving to [deg]: ");
	printAngles(angles);
	
	for (uint8_t i=0; i<_numSmartServos; i++) moveToAngle(i, angles[i], speedRPM);
}

void morobotClass::moveAngle(uint8_t servoId, long angle){
	if (checkIfAngleValid(servoId, getActAngle(servoId)+angle) == true) smartServos.move(servoId+1, angle, _speedRPM);
}

void morobotClass::moveAngle(uint8_t servoId, long angle, uint8_t speedRPM){
	if (checkIfAngleValid(servoId, getActAngle(servoId)+angle) == true) smartServos.move(servoId+1, angle, speedRPM);
}

void morobotClass::moveAngles(long angles[]){
	waitUntilIsReady();
	setBusy();
	Serial.print("Moving [deg]: ");
	printAngles(angles);

	for (uint8_t i=0; i<_numSmartServos; i++) moveAngle(i+1, angles[i]);
}

void morobotClass::moveAngles(long angles[], uint8_t speedRPM){
	waitUntilIsReady();
	setBusy();
	Serial.print("Moving [deg]: ");
	printAngles(angles);

	for (uint8_t i=0; i<_numSmartServos; i++) moveAngle(i+1, angles[i], speedRPM);
}

bool morobotClass::moveToPosition(float x, float y, float z){
	if (calculateAngles(x, y, z) == false) return false;
	
	long goalAngles[_numSmartServos];
	for (uint8_t i=0; i<_numSmartServos; i++) moveToAngle(i, _goalAngles[i]);
	return true;
}

bool morobotClass::moveXYZ(float xOffset, float yOffset, float zOffset){
	updateCurrentXYZ();
	return moveToPosition(_actPos[0]+xOffset, _actPos[1]+yOffset, _actPos[2]+zOffset);
}

bool morobotClass::moveInDirection(char axis, float value){
	updateCurrentXYZ();
	float goalxyz[3];
	goalxyz[0] = _actPos[0];
	goalxyz[1] = _actPos[1];
	goalxyz[2] = _actPos[2];
	
	if (axis == 'x') goalxyz[0] = goalxyz[0]+value;
	else if (axis == 'y') goalxyz[1] = goalxyz[1]+value;
	else if (axis == 'z') goalxyz[2] = goalxyz[2]+value;
	return moveToPosition(goalxyz[0], goalxyz[1], goalxyz[2]);
}

/* HELPER */    

void morobotClass::printAngles(long angles[]){
	for (uint8_t i=0; i<_numSmartServos; i++) {
		Serial.print(angles[i]);
		if (i != _numSmartServos-1) Serial.print(", ");
		else Serial.println(" ");
	}
}
