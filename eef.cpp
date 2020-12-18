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
	_closingDirectionIsPositive = true;
	
	setSpeed(50);
	setGearRatio(12.34);
	setParams(0, -300, 0, -300);	//float degClosed, float degOpen, float degCloseLimit, float degOpenLimit
	setTCPoffset(0, 0, -25);	// Store TCP-Offset

	morobot->setTCPoffset(_tcpOffset[0], _tcpOffset[1], _tcpOffset[1]);
	//morobot->smartServos.setInitAngle(_servoID+1, 0, 15)
	Serial.println("Gripper connected to robot");
}

void gripper::begin(int8_t servoPin){
	_gripperType = 1;
	_servoPin = servoPin;
	_servoID = -1;
	_closingDirectionIsPositive = true;

	setSpeed(50);
	setParams(0, 90, 0, 100);
	setTCPoffset(30, 0, -25);

	morobot->setTCPoffset(_tcpOffset[0], _tcpOffset[1], _tcpOffset[1]);
	Serial.println("Gripper connected to robot");
}

bool gripper::autoCalibrate(){
	Serial.println("AUTOCALIBRATING GRIPPER");
	bool returnValue = true;
	
	if (_gripperType == 0){
		Serial.println("Closing...");
		if (closeToForce() == true){
			Serial.println("Closed");
			_degClosed = getCurrentOpeningAngle();
			_degCloseLimit = getCurrentOpeningAngle();
			/*_closingDirectionIsPositive = !_closingDirectionIsPositive;
			Serial.println("Opening");
			if (closeToForce() == true){
				Serial.println("Open");
				_degOpen = getCurrentOpeningAngle();
				_degOpenLimit = getCurrentOpeningAngle();				
			} else {
				_closingDirectionIsPositive = !_closingDirectionIsPositive;
				returnValue = false;
			}
			_closingDirectionIsPositive = !_closingDirectionIsPositive;*/
		} else {
			returnValue = false;
		}
	} else {
		returnValue = false;
	}
	if (returnValue == false) {
		Serial.println("ERROR: Calibration failed!");
		return false;
	}
	morobot->smartServos.setZero(_servoID+1);
	_degOpen = degClosed + 350;
	_degOpenLimit = degCloseLimit + 400;
	Serial.print("Calibration successful! New Limits: Closed at ");
	Serial.print(_degCloseLimit);
	Serial.print(", Opened at ");
	Serial.println(_degOpenLimit);
	close();
	return true;
}

void gripper::setParams(float degClosed, float degOpen, float degCloseLimit, float degOpenLimit){
	_degClosed = degClosed;
	_degOpen = degOpen;
	_degCloseLimit = degCloseLimit;
	_degOpenLimit = degOpenLimit;
}

void gripper::setGearRatio(float gearRatio){
	_gearRatio = gearRatio;
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
	morobot->setTCPoffset(_tcpOffset[0], _tcpOffset[1], _tcpOffset[1]);
}

void gripper::close(){
	moveToAngle(_degClosed, _speed[1]);
	_isOpen = false;
	_isClosed = true;
}

void gripper::open(){
	moveToAngle(_degOpen, _speed[0]);
	_isOpen = true;
	_isClosed = false;
}

bool gripper::moveToAngle(float angle, uint8_t speed){
	if ((_closingDirectionIsPositive == true && (angle > _degCloseLimit || angle < _degOpenLimit)) || (_closingDirectionIsPositive == false && (angle < _degCloseLimit || angle > _degOpenLimit))){
		Serial.println("ERROR: ANGLE OUT OF LIMIT");
		return false;
	}
	if (_gripperType == 0){
		morobot->smartServos.moveTo(_servoID+1, angle, speed);
		return waitUntilFinished();
	} else if (_gripperType == 1) {
		// TODO SERVO-CODE
	}	
}

bool gripper::moveToWidth(float width, uint8_t speed){
	float angle = width * _gearRatio;
	moveToAngle(angle, speed);	
}

bool gripper::closeToForce(float maxCurrent){
	float closingStep = -3;
	if (_closingDirectionIsPositive == true) closingStep = closingStep * -1;
	if (_gripperType == 0){
		unsigned long startTime = millis();
		while(true){
			morobot->smartServos.move(_servoID+1, closingStep, 20);
			if (morobot->smartServos.getCurrentRequest(_servoID+1) > maxCurrent) {
				delay(100);
				if (morobot->smartServos.getCurrentRequest(_servoID+1) > maxCurrent) return true;	// If after 20ms there is still too much current - ausreisser
			}
			
			// Stop if the gripper is not finished after a timeout occurs
			if ((millis() - startTime) > TIMEOUT_DELAY_GRIPPER) {
				Serial.println("TIMEOUT OCCURED WHILE WAITING FOR GRIPPER TO FINISH MOVEMENT!");
				return false;
			}
		}
	}
	return false;
}

float gripper::getCurrentOpeningAngle(){
	return morobot->smartServos.getAngleRequest(_servoID+1);
	// TODO: Solution for Servo
}

float gripper::getCurrentOpeningWidth(){
	if (_gripperType == 0) return getCurrentOpeningAngle() / _gearRatio;
	else return -1;
}

bool gripper::isClosed(){
	return _isClosed;
}
bool gripper::isOpen(){
	return _isOpen;
}

bool gripper::waitUntilFinished(){
	unsigned long startTime = millis();
	while (true){
		// Check if the gripper moves
		long startPos = getCurrentOpeningAngle();
		delay(50);
		if (startPos == getCurrentOpeningAngle()) return true;
		//Serial.println(morobot->smartServos.getCurrentRequest(_servoID+1));
		// Stop waiting if the gripper is not finished after a timeout occurs
		if ((millis() - startTime) > TIMEOUT_DELAY_GRIPPER) {
			Serial.println("TIMEOUT OCCURED WHILE WAITING FOR GRIPPER TO FINISH MOVEMENT!");
			return false;
		}
	}
}