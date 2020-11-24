#include <Arduino.h>
#include <morobot.h>
#include <SoftwareSerial.h>
#include <MakeblockSmartServo.h>

//TODO: Implement kinematics and EEF-Frame
//TODO: Implement blocking methods

morobotClass::morobotClass(){
	
}

void morobotClass::begin(uint8_t numSmartServos, const char* stream){
	Serial.begin(115200);
	
	if (stream == "Serial1") {
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
	
	if (numSmartServos > NUM_MAX_SERVOS){
		Serial.print("Too many motors! Maximum number of motors: ");
		Serial.println(NUM_MAX_SERVOS);
		return NULL;
	}
	_numSmartServos = numSmartServos;
	positionReached[numSmartServos];
	
    smartServos.beginserial(_port);
	//delay(5);
    smartServos.assignDevIdRequest();
    delay(50);

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
	for (uint8_t i=0; i<_numSmartServos; i++) positionReached[i] = false;
}

void morobotClass::setIdle(){
	for (uint8_t i=0; i<_numSmartServos; i++) positionReached[i] = true;
}

bool morobotClass::isReady(){
	for (uint8_t i=0; i<_numSmartServos; i++) {
		/*Serial.print("Testing ");
		Serial.print(i);
		Serial.println(positionReached[i]);*/
		if (positionReached[i] == false) {
			if (checkIfMotorMoves(i+1) == false) continue;
			//Serial.print(i);
			//Serial.println(": Got no callback yet and motor sill moves");
			return false;
		}
	}
	return true;
}

void morobotClass::waitUntilIsReady(){
	unsigned long startTime = millis();
	while (true){
		delay(100);
		smartServos.smartServoEventHandle();
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
	delay(100);
	if (startPos != getActAngle(servoId)) return true;
	return false;
}


/* GETTERS */

long morobotClass::getActAngle(uint8_t servoId){
	return smartServos.getAngleRequest(servoId+1);
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
	smartServos.moveTo(servoId+1, angle, _speedRPM, callback_done);
}

void morobotClass::moveToAngle(uint8_t servoId, long angle, uint8_t speedRPM){
	smartServos.moveTo(servoId+1, angle, speedRPM, callback_done);
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
	smartServos.move(servoId+1, angle, _speedRPM, callback_done);
}

void morobotClass::moveAngle(uint8_t servoId, long angle, uint8_t speedRPM){
	smartServos.move(servoId+1, angle, speedRPM, callback_done);
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


/* HELPER */

void morobotClass::printAngles(long angles[]){
	for (uint8_t i=0; i<_numSmartServos; i++) {
		Serial.print(angles[i]);
		if (i != _numSmartServos-1) Serial.print(", ");
		else Serial.println(" ");
	}
}

//morobotClass morobot;

void callback_done(uint8_t servoNum){
	//morobot.positionReached[servoNum-1] = true;
}