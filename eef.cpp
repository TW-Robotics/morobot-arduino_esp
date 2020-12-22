/**
 *  \class 	gripper, binaryEEF
 *  \brief 	Endeffector class for all kinds of endeffectors for the morobots
 *  @file 	eef.cpp
 *  @author	Johannes Rauer FHTW
 *  @date	2020/12/18
 *  \par Method List:
 *  	gripper:
 *  		public:
 *  			gripper();
 *  			gripper(uint8_t servoPin);
 *  			void setParams(float degOpen, float degClosed, float degLimitMin, float degLimitMax);
 *  			void setTCPoffset(float xOffset, float yOffset, float zOffset);
 *  			void close();
 *  			void open();
 *  			bool moveToWidth(float width);
 *  			bool moveToAngle(float angle);
 * 				bool isClosed();
 *				bool isOpen();
 *  	binaryEEF:
 *  		public:
 *  			binaryEEF(uint8_t pin);
 *  			void activate();
 *  			void deactivate();
 *  			bool isActivated();
 *  			bool isDeactivated();
 */
 
#include <eef.h>

gripper::gripper(morobotClass* morobotToAttachTo){
	_isOpen = false;
	_isClosed = false;
	
	morobot = morobotToAttachTo;
}

void gripper::begin(){
	_gripperType = 0;
	_servoPin = -1;
	_servoID = morobot->getNumSmartServos();			// e.g. robot has 3 smart servos (ID 0,1,2) -> gripper gets id 3
	
	setSpeed(50);
	setParams(0, -500, 0, -550);	//float degClosed, float degOpen, float degCloseLimit, float degOpenLimit
	setTCPoffset(0, 0, -38);		// Store TCP-Offset

	Serial.println("Gripper connected to robot");
}

void gripper::begin(int8_t servoPin){
	_gripperType = 1;
	_servoPin = servoPin;
	_servoID = -1;

	setSpeed(50);
	setParams(65, 140, 50, 155, 1.0, 0);		//values for servomotor -> degClosed = 0 degrees in operation. float degClosed, float degOpen, float degCloseLimit, float degOpenLimit; Values of servo
	setTCPoffset(0, 0, -18);
	
	servo.attach(servoPin);
	open();

	Serial.println("Gripper connected to robot");
}

bool gripper::autoCalibrate(){
	Serial.println("AUTOCALIBRATING GRIPPER");
	bool returnValue = true;
	
	if (_gripperType == 0){
		Serial.println("Closing...");
		if (closeToForce() == true){
			Serial.println("Closed");
		} else {
			returnValue = false;
		}
	} else {
		returnValue = false;
		functionNotImplementedError();
	}
	if (returnValue == false) {
		Serial.println("ERROR: Calibration failed!");
		return false;
	}
	morobot->smartServos.setZero(_servoID+1);
	int8_t positiveFactor = 1;
	if (_closingDirectionIsPositive == true) positiveFactor = -1;
	setParams(getCurrentOpeningAngle(), getCurrentOpeningAngle() + positiveFactor * 500, getCurrentOpeningAngle(), getCurrentOpeningAngle() + positiveFactor * 550);

	Serial.print("Calibration successful! New Limits: Closed at ");
	Serial.print(_degClosed);
	Serial.print(", Opened at ");
	Serial.println(_degOpen);
	close();
	return true;
}

void gripper::setParams(float degClosed, float degOpen, float degCloseLimit, float degOpenLimit, float gearRatio=7.87, float closingWidthOffset=101.2){
	_degClosed = degClosed;
	_degOpen = degOpen;
	_degCloseLimit = degCloseLimit;
	_degOpenLimit = degOpenLimit;
	_gearRatio = gearRatio;
	_closingWidthOffset = closingWidthOffset;
	
	if (_degClosed > _degOpen) _closingDirectionIsPositive = true;
	else _closingDirectionIsPositive = false;
}

void gripper::setSpeed(uint8_t speed){
	setSpeed(speed, speed);
}

void gripper::setSpeed(uint8_t speedOpening, uint8_t speedClosing){
	_speed[0] = speedOpening;
	_speed[1] = speedClosing;
}

void gripper::setTCPoffset(float xOffset, float yOffset, float zOffset){
	_tcpOffset[0] = xOffset;
	_tcpOffset[1] = yOffset;
	_tcpOffset[2] = zOffset;
	morobot->setTCPoffset(_tcpOffset[0], _tcpOffset[1], _tcpOffset[2]);
}

void gripper::close(){
	float angle = _degClosed;
	if (_gripperType == 1) angle = 0;
	moveToAngle(angle, _speed[1]);
	_isOpen = false;
	_isClosed = true;
}

void gripper::open(){
	float angle = _degOpen;
	if (_gripperType == 1) angle = _degOpen / _gearRatio - _degClosed;
	moveToAngle(angle, _speed[0]);
	_isOpen = true;
	_isClosed = false;
}

bool gripper::moveAngle(float angle, uint8_t speed){
	return moveToAngle(getCurrentOpeningAngle() + angle, speed);
}

bool gripper::moveToAngle(float angle, uint8_t speed){
	if (_gripperType == 0){
		if (checkIfAngleValid(angle) != true) return false;
		float oldAngle = getCurrentOpeningAngle();
		morobot->smartServos.moveTo(_servoID+1, angle, speed);
		_currentAngle = getCurrentOpeningAngle();
		if (abs(oldAngle-angle) > 10) return waitUntilFinished();
	} else if (_gripperType == 1) {
		angle = angle * _gearRatio + _degClosed;
		if (checkIfAngleValid(angle) != true) return false;
		
		float timeForOneDegree = 60 / (float)speed / 360;
		float angleStep = 1;
		float angleTemp = getCurrentOpeningAngle() * _gearRatio + _degClosed;
		bool goalIsBiggerThanCurrent = true;
		if (angle <= angleTemp) goalIsBiggerThanCurrent = false;
		
		while(1){
			if (goalIsBiggerThanCurrent == true){
				angleTemp = angleTemp + angleStep;
				if (angleTemp > angle) break;
			} else if (goalIsBiggerThanCurrent == false){
				angleTemp = angleTemp - angleStep;
				if (angleTemp < angle) break;
			}
			servo.write(angleTemp);
			delay(timeForOneDegree * angleStep * 1000);
		}
		_currentAngle = angle / _gearRatio - _degClosed;
	}
	_isOpen = false;
	_isClosed = false;
	return true;
}

bool gripper::moveWidth(float width, uint8_t speed){
	if (_gripperType == 0) return moveToWidth(getCurrentOpeningWidth() + width, speed);
	else return functionNotImplementedError();
}

bool gripper::moveToWidth(float width, uint8_t speed){
	if (_gripperType == 0){ 
		int8_t positiveFactor = 1;
		if (_closingDirectionIsPositive == true) positiveFactor = -1;
		
		float angle = positiveFactor * (width - _closingWidthOffset) * _gearRatio;
		moveToAngle(angle, speed);
	} else {
		return functionNotImplementedError();
	}
}

bool gripper::closeToForce(float maxCurrent){
	if (_gripperType == 0){
		float closingStep = -5;
		if (_closingDirectionIsPositive == true) closingStep = closingStep * -1;
		unsigned long startTime = millis();
		while(true){
			morobot->smartServos.move(_servoID+1, closingStep, 20);
			if (morobot->smartServos.getCurrentRequest(_servoID+1) > maxCurrent) {
				delay(20);
				if (morobot->smartServos.getCurrentRequest(_servoID+1) > maxCurrent) {
					Serial.println("Grasped object");
					_currentAngle = getCurrentOpeningAngle();
					_isOpen = false;
					_isClosed = true;
					return true;	// If after 20ms there is still too much current - ausreisser
				}
			}
			
			// Stop if the gripper is not finished after a timeout occurs
			if ((millis() - startTime) > TIMEOUT_DELAY_GRIPPER) {
				Serial.println("TIMEOUT OCCURED WHILE WAITING FOR GRIPPER TO FINISH MOVEMENT!");
				return false;
			}
		}
	} else {
		return functionNotImplementedError();
	}
	return false;
}

float gripper::getCurrentOpeningAngle(){
	if (_gripperType == 0) return morobot->smartServos.getAngleRequest(_servoID+1);
	else if (_gripperType == 1) return _currentAngle;
	else return functionNotImplementedError();
}

float gripper::getCurrentOpeningWidth(){
	if (_gripperType == 0) return _closingWidthOffset + abs(getCurrentOpeningAngle() / _gearRatio);
	else return functionNotImplementedError();
}

bool gripper::isClosed(){
	return _isClosed;
}
bool gripper::isOpen(){
	return _isOpen;
}

bool gripper::checkIfAngleValid(float angle){
	if ((_closingDirectionIsPositive == true && (angle > _degCloseLimit || angle < _degOpenLimit)) || (_closingDirectionIsPositive == false && (angle < _degCloseLimit || angle > _degOpenLimit))){
		Serial.println("ERROR: ANGLE OUT OF LIMIT");
		Serial.println(angle);
		return false;
	}
	return true;
}

bool gripper::waitUntilFinished(){
	unsigned long startTime = millis();
	while (true){
		// Check if the gripper moves
		long startPos = getCurrentOpeningAngle();
		delay(50);
		if (startPos == getCurrentOpeningAngle()) return true;
		// Stop waiting if the gripper is not finished after a timeout occurs
		if ((millis() - startTime) > TIMEOUT_DELAY_GRIPPER) {
			Serial.println("TIMEOUT OCCURED WHILE WAITING FOR GRIPPER TO FINISH MOVEMENT!");
			return false;
		}
	}
}

bool gripper::functionNotImplementedError(){
	Serial.println("ERROR: Function not implemented for this gripper type");
	return false;
}