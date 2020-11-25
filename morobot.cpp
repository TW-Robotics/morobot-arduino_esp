#include <Arduino.h>
#include <morobot.h>
#include <SoftwareSerial.h>
#include <MakeblockSmartServo.h>

//TODO: Implement kinematics and EEF-Frame; updateCurrentXYZ, moveInOneDirection
//TODO: Implement Limits (check if valid)

morobotClass::morobotClass(){
	
}

void morobotClass::begin(uint8_t numSmartServos, const char* stream){
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
	
	if (numSmartServos > NUM_MAX_SERVOS){
		Serial.print("Too many motors! Maximum number of motors: ");
		Serial.println(NUM_MAX_SERVOS);
		return NULL;
	}
	_numSmartServos = numSmartServos;
	positionReached[numSmartServos];
	
    smartServos.beginSerial(_port);
	//delay(5);
    smartServos.assignDevIdRequest();
    delay(50);
	
	setTCPpos(0, 0, 0);

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

void morobotClass::setTCPpos(float xOffset, float yOffset, float zOffset){
	_tcpPos[0] = xOffset;
	_tcpPos[1] = yOffset;
	_tcpPos[2] = zOffset;
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
		if (positionReached[i] == false) {
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
	delay(100);
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
	smartServos.moveTo(servoId+1, angle, _speedRPM);
}

void morobotClass::moveToAngle(uint8_t servoId, long angle, uint8_t speedRPM){
	smartServos.moveTo(servoId+1, angle, speedRPM);
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
	smartServos.move(servoId+1, angle, _speedRPM);
}

void morobotClass::moveAngle(uint8_t servoId, long angle, uint8_t speedRPM){
	smartServos.move(servoId+1, angle, speedRPM);
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
	for (uint8_t i=0; i<_numSmartServos; i++) goalAngles[i] = _goalAngles[i];
	moveToAngles(goalAngles);
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

bool morobotClass::calculateAngles(float x, float y, float z){
	// TODO: Change for other robots
    float a = 47.0;		// From mounting to first axis
    float b = 92.9;		// From first axis to second axis
    float c = 72.79;	// From second axis to center of flange
	float gearRatio = 16.25;		// Turn motor of linear axis by gearRatio degrees to move it 1 mm
    	
    x = x-a;	// Base is in x-orientation --> Just subtract its length from x-coordinate
	z = z-_tcpPos[2];
    
	// Calculate new length and angle of last axis (since eef is connected to it statically)
    float c_new = sqrt(pow(_tcpPos[1],2) + pow(c+_tcpPos[0],2));
    float beta = asin(fabs(_tcpPos[1]/c_new));
	
	// Calculate angle for 2nd axis
	float phi2 = acos((pow(x,2) + pow(y,2) - pow(b,2) - pow(c_new,2)) / (2*b*c_new));
	phi2 = (phi2 - beta) * 180/M_PI;
	
	// Calculate angle for 1st axis
	float gamma = atan2(y, x);
	float phi1 = acos((pow(x,2) + pow(y,2) + pow(b,2) - pow(c_new,2))/(2*b*sqrt(pow(x,2) + pow(y,2))));
	phi1 = gamma + phi1;
    phi1 = phi1 * 180/M_PI;
	
	// Calculate angle for 3rd axis (z-direction)
	float phi3 = z*gearRatio;
	
	// Check if angles are valid
	if (isnan(phi1) || isnan(phi2) || isnan(phi3) || fabs(phi1) > 100 || fabs(phi2) > 100 || phi3 < 0 || phi3 > 780){
		Serial.print("Position cannot be reached: ");
		Serial.print(phi1);
		Serial.print(", ");
		Serial.print(phi2);
		Serial.print(", ");
		Serial.println(phi3);
		Serial.print(x);
		Serial.print(", ");
		Serial.print(y);
		Serial.print(", ");
		Serial.println(z);
		return false;
	}
	_goalAngles[0] = phi1;
	_goalAngles[1] = phi2;
	_goalAngles[2] = phi3;
	
	/*Serial.print("Calculated Angles: ");
	Serial.print(_goalAngles[0]);
	Serial.print(", ");
	Serial.print(_goalAngles[1]);
	Serial.print(", ");
	Serial.println(_goalAngles[2]);*/
	
	return true;
}

void morobotClass::updateCurrentXYZ(){
	setBusy();
	waitUntilIsReady();
	
    float a = 47.0;		// From mounting to first axis
    float b = 92.9;		// From first axis to second axis
    float c = 72.79;	// From second axis to center of flange
	float gearRatio = 16.25;		// Turn motor of linear axis by gearRatio degrees to move it 1 mm
	
	long actAngles[_numSmartServos];
	for (uint8_t i=0; i<_numSmartServos; i++) actAngles[i] = getActAngle(i);
	
	// Calculate new length and angle of last axis (since eef is connected to it statically)
    float c_new = sqrt(pow(_tcpPos[1],2) + pow(c+_tcpPos[0],2));
    float beta = asin(fabs(_tcpPos[1]/c_new));
	
	float xnb = b*cos((float)actAngles[0]*M_PI/180);
    float ynb = b*sin((float)actAngles[0]*M_PI/180);
    float delta = asin(xnb/b);
    float phi2s = M_PI-delta-M_PI/2-((float)actAngles[1]*M_PI/180+beta);
    float xncn = c_new*cos(phi2s);
    float yncn = c_new*sin(phi2s);
    
	_actPos[0] = a + xnb + xncn;
    _actPos[1] = ynb + yncn;
	_actPos[2] = actAngles[2]/gearRatio;
	
	/*Serial.print("Calculated Position: ");
	Serial.println(_actPos[0]);
	Serial.println(_actPos[1]);
	Serial.println(_actPos[2]);*/
}
    

void morobotClass::printAngles(long angles[]){
	for (uint8_t i=0; i<_numSmartServos; i++) {
		Serial.print(angles[i]);
		if (i != _numSmartServos-1) Serial.print(", ");
		else Serial.println(" ");
	}
}
