/**
 *  \class 	gripper, binaryEEF
 *  \brief 	Endeffector class for all kinds of endeffectors for the morobots
 *  @file 	eef.cpp
 *  @author	Johannes Rauer FHTW
 *  @date	2020/12/22
 *  \par Method List:
 *  	gripper:
			public:
				gripper(morobotClass* morobotToAttachTo);
				void begin();
				void begin(int8_t servoPin);
				bool autoCalibrate();
				void setParams(float degClosed, float degOpen, float degCloseLimit, float degOpenLimit, float gearRatio=7.87, float closingWidthOffset=101.2);
				void setSpeed(uint8_t speed);
				void setSpeed(uint8_t speedOpening, uint8_t speedClosing);
				void setTCPoffset(float xOffset, float yOffset, float zOffset);
				void close();
				void open();
				bool moveAngle(float angleInc, uint8_t speed=50);
				bool moveToAngle(float angle, uint8_t speed=50);
				bool moveWidth(float width, uint8_t speed=50);
				bool moveToWidth(float width, uint8_t speed=50);
				bool closeToForce(float maxCurrent=70);
				float getCurrentOpeningAngle();
				float getCurrentOpeningWidth();
				bool isClosed();
				bool isOpened();
			private:
				bool checkIfAngleValid(float angle);
				bool waitUntilFinished();
				bool functionNotImplementedError();
 *  	binaryEEF:
 *  		public:
 *  			binaryEEF(uint8_t pin);
 *  			void activate();
 *  			void deactivate();
 *  			bool isActivated();
 *  			bool isDeactivated();
 */
 
#include <eef.h>

/**
 *  \brief Constructor of gripper class
 *  \param [in] morobotToAttachTo Pointer to morobot object the gripper is attached to (relevant for smart servo control)
 */
gripper::gripper(morobotClass* morobotToAttachTo){
	_isOpened = false;
	_isClosed = false;
	
	morobot = morobotToAttachTo;
}

/**
 *  \brief Initialices all important variables for smartservo-gripper
 *  \details This function must be called in the setup-function of the code.
 *  This function changes the TCP-Position of the morobot
 */
void gripper::begin(){
	_gripperType = 0;
	_servoID = morobot->getNumSmartServos();	// e.g. robot has 3 smart servos (ID 0,1,2) -> gripper gets id 3
	
	setSpeed(50);
	setParams(0, -500, 0, -550);	// degClosed, degOpen, degCloseLimit, degOpenLimit
	setTCPoffset(0, 0, -38);		// Store TCP-Offset and change TCP-Offset of morobot

	Serial.println("Gripper connected to robot");
}

/**
 *  \brief Initialices all important variables for servo-gripper
 *  \param [in] servoPin The pin of the arduino the servo motor is connected to
 *  \details This function must be called in the setup-function of the code
 *  This function changes the TCP-Position of the morobot
 */
void gripper::begin(int8_t servoPin){
	_gripperType = 1;
	_servoID = -1;

	setSpeed(50);
	setParams(65, 140, 50, 155, 1.0, 0);	// values for servomotor -> degClosed is defined as 0 degrees in real life. degClosed, degOpen, degCloseLimit, degOpenLimit, gearRatio, openWidthOffset
	setTCPoffset(0, 0, -18);				// Store TCP-Offset and change TCP-Offset of morobot
	
	servo.attach(servoPin);
	open();

	Serial.println("Gripper connected to robot");
}

/**
 *  \brief Autonomous calibration for smart servo gripper
 *  \return Returns true is calibration was successful
 *  \details Closes gripper until a defined force (current) appears (= closed position), sets the motor zero and adds the default values for an open gripper 
 */
bool gripper::autoCalibrate(){
	Serial.println("Autocalibrating Gripper");
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
	
	int8_t positiveFactor = 1;	// Check if the gripper is moving positively when closing so the values for opened/closed are correct
	if (_closingDirectionIsPositive == true) positiveFactor = -1;
	setParams(getCurrentOpeningAngle(), getCurrentOpeningAngle() + positiveFactor * 500, getCurrentOpeningAngle(), getCurrentOpeningAngle() + positiveFactor * 550);

	Serial.print("Calibration successful! New Limits: Closed at ");
	Serial.print(_degClosed);
	Serial.print(", Opened at ");
	Serial.println(_degOpen);
	close();
	return true;
}

/**
 *  \brief Sets all important parameters of the gripper
 *  \param [in] degClosed Degrees at which the gripper is closed (used when calling close())
 *  \param [in] degOpen Degrees at which the gripper is opened (used when calling open())
 *  \param [in] degCloseLimit Gripper will not close above this limit (would harm hardware)
 *  \param [in] degOpenLimit Gripper will not open above this limit (would harm hardware)
 *  \param [in] gearRatio The gearRatio defines how many mm (or degrees) the gripper moves per degree motor movement
 *  \param [in] closingWidthOffset Opening width of the gripper when it is completely closed (for parallel grippers only
 */
void gripper::setParams(float degClosed, float degOpen, float degCloseLimit, float degOpenLimit, float gearRatio=7.87, float closingWidthOffset=101.2){
	_degClosed = degClosed;
	_degOpen = degOpen;
	_degCloseLimit = degCloseLimit;
	_degOpenLimit = degOpenLimit;
	_gearRatio = gearRatio;
	_closingWidthOffset = closingWidthOffset;
	
	if (_degClosed > _degOpen) _closingDirectionIsPositive = true;
	else _closingDirectionIsPositive = false;

	Serial.print("New Limits: Closed at ");
	Serial.print(_degClosed);
	Serial.print(", Opened at ");
	Serial.println(_degOpen);	
}

/**
 *  \brief Sets the opening and closing speed of the gripper
 *  \param [in] speed Opening and closing speed in RPM (max. 50)
 */
void gripper::setSpeed(uint8_t speed){
	setSpeed(speed, speed);
}

/**
 *  \brief Sets the opening and closing speed of the gripper independently
 *  \param [in] speedOpening Opening speed in RPM (max. 50)
 *  \param [in] speedClosing Closing speed in RPM (max. 50)
 */
void gripper::setSpeed(uint8_t speedOpening, uint8_t speedClosing){
	_speed[0] = speedOpening;
	_speed[1] = speedClosing;
}

/**
 *  \brief Sets the TCP-Offset of the gripper and therefore also of the morobot
 *  \param [in] xOffset Offset in direction x
 *  \param [in] yOffset Offset in direction y
 *  \param [in] zOffset Offset in direction z
 */
void gripper::setTCPoffset(float xOffset, float yOffset, float zOffset){
	_tcpOffset[0] = xOffset;
	_tcpOffset[1] = yOffset;
	_tcpOffset[2] = zOffset;
	morobot->setTCPoffset(_tcpOffset[0], _tcpOffset[1], _tcpOffset[2]);
}

/**
 *  \brief Moves the gripper to its closing position (defined by _degClosed in setParams())
 */
void gripper::close(){
	float angle = _degClosed;
	if (_gripperType == 1) angle = 0;
	moveToAngle(angle, _speed[1]);
	_isOpened = false;
	_isClosed = true;
}

/**
 *  \brief Moves the gripper to its opening position (defined by _degOpened in setParams())
 */
void gripper::open(){
	float angle = _degOpen;
	if (_gripperType == 1) angle = _degOpen / _gearRatio - _degClosed;	// Calculate from real degrees into motor degrees
	moveToAngle(angle, _speed[0]);
	_isOpened = true;
	_isClosed = false;
}

/**
 *  \brief Moves the gripper relatively by a defined angle
 *  \param [in] angleInc Angle to move the gripper by
 *  \param [in] speed Speed to use for movement (Default: maximum speed)
 *  \return Returns true if movement has been successful
 */
bool gripper::moveAngle(float angle, uint8_t speed){
	return moveToAngle(getCurrentOpeningAngle() + angle, speed);
}

/**
 *  \brief Moves the gripper to a defined angle (absolute)
 *  \param [in] angle Angle to move the gripper to
 *  \param [in] speed Speed to use for movement (Default: maximum speed)
 *  \return Returns true if movement has been successful
 */
bool gripper::moveToAngle(float angle, uint8_t speed){
	if (_gripperType == 0){
		if (checkIfAngleValid(angle) != true) return false;			// Check if angle is valid
		float oldAngle = getCurrentOpeningAngle();
		morobot->smartServos.moveTo(_servoID+1, angle, speed);
		_currentAngle = getCurrentOpeningAngle();
		if (abs(oldAngle-angle) > 10) return waitUntilFinished();	// Only wait for bigger movements
	} else if (_gripperType == 1) {
		angle = angle * _gearRatio + _degClosed;					// Calculate angle in motor-degrees
		if (checkIfAngleValid(angle) != true) return false;
		
		float timeForOneDegree = 60 / (float)speed / 360;
		float angleStep = 1;
		float angleTemp = getCurrentOpeningAngle() * _gearRatio + _degClosed;	// Calculate current angle in motor-degrees
		
		bool goalIsBiggerThanCurrent = true;						// Check orientation direction depending on values
		if (angle <= angleTemp) goalIsBiggerThanCurrent = false;
		
		while(1){
			if (goalIsBiggerThanCurrent == true){
				angleTemp = angleTemp + angleStep;
				if (angleTemp > angle) break;
			} else if (goalIsBiggerThanCurrent == false){
				angleTemp = angleTemp - angleStep;
				if (angleTemp < angle) break;
			}
			servo.write(angleTemp);									// Move only a little bit and
			delay(timeForOneDegree * angleStep * 1000);				// wait to stick to defined speed
		}
		_currentAngle = angle / _gearRatio - _degClosed;			// Store new angle (in real degrees)
	}
	_isOpened = false;
	_isClosed = false;
	return true;
}

/**
 *  \brief Moves the gripper relatively by a defined width
 *  \param [in] width Width to move the gripper by
 *  \param [in] speed Speed to use for movement (Default: maximum speed)
 *  \return Returns true if movement has been successful
 */
bool gripper::moveWidth(float width, uint8_t speed){
	if (_gripperType == 0) return moveToWidth(getCurrentOpeningWidth() + width, speed);
	else return functionNotImplementedError();
}

/**
 *  \brief Moves the gripper to a defined width (absolute)
 *  \param [in] width Width to move the gripper to
 *  \param [in] speed Speed to use for movement (Default: maximum speed)
 *  \return Returns true if movement has been successful
 */
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

/**
 *  \brief Closes the gripper until a defined fore (current) is reached
 *  \param [in] maxCurrent Current at which the movement should stop
 *  \return Returns true if movement has been successful
 */
bool gripper::closeToForce(float maxCurrent){
	if (_gripperType == 0){
		float closingStep = -5;
		if (_closingDirectionIsPositive == true) closingStep = closingStep * -1;		// Define rotation direction
		unsigned long startTime = millis();
		while(true){
			morobot->smartServos.move(_servoID+1, closingStep, 20);						// Move the motor a little bit
			if (morobot->smartServos.getCurrentRequest(_servoID+1) > maxCurrent) {		// Check the current
				delay(20);
				if (morobot->smartServos.getCurrentRequest(_servoID+1) > maxCurrent) {	// If after 20ms there is still too much current, the final position is reached (outlier detection)
					Serial.println("Grasped object");
					_currentAngle = getCurrentOpeningAngle();
					_isOpened = false;
					_isClosed = true;
					return true;	
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

/**
 *  \brief Calculates and return current opening angle of the gripper
 *  \return Current opening angle of the gripper
 */
float gripper::getCurrentOpeningAngle(){
	if (_gripperType == 0) {
		_currentAngle = morobot->smartServos.getAngleRequest(_servoID+1);
		return _currentAngle;
	} else if (_gripperType == 1) {
		return _currentAngle;
	}
	return functionNotImplementedError();
}

/**
 *  \brief Calculates and return current opening width of the gripper. Only implemented for smart-servo (parallel-gripper).
 *  \return Current opening width of the gripper
 */
float gripper::getCurrentOpeningWidth(){
	if (_gripperType == 0) return _closingWidthOffset + abs(getCurrentOpeningAngle() / _gearRatio);
	else return functionNotImplementedError();
}

/**
 *  \brief Checks if the gripper is closed
 *  \return True if gripper is closed, false otherwise
 */
bool gripper::isClosed(){
	return _isClosed;
}

/**
 *  \brief Checks if the gripper is opened
 *  \return True if gripper is opened, false otherwise
 */
bool gripper::isOpened(){
	return _isOpened;
}

/**
 *  \brief Checks if a given angle can be reached
 *  \param [in] angle Angle to drive to
 *  \return True if angle can be reached, false otherwise
 */
bool gripper::checkIfAngleValid(float angle){
	if ((_closingDirectionIsPositive == true && (angle > _degCloseLimit || angle < _degOpenLimit)) || (_closingDirectionIsPositive == false && (angle < _degCloseLimit || angle > _degOpenLimit))){
		Serial.println("ERROR: ANGLE OUT OF LIMIT");
		Serial.println(angle);
		return false;
	}
	return true;
}

/**
 *  \brief Makes sure the program does not continue until smart-servo-gripper has stopped moving
 *  \return True is the gripper reached the target position, false otherwise
 */
bool gripper::waitUntilFinished(){
	unsigned long startTime = millis();
	while (true){
		long startPos = getCurrentOpeningAngle();
		delay(50);
		if (startPos == getCurrentOpeningAngle()) return true;	// If after 50ms the position is still the same, the gripper has reached the goal
		// Stop waiting if the gripper is not finished after a timeout occurs
		if ((millis() - startTime) > TIMEOUT_DELAY_GRIPPER) {
			Serial.println("TIMEOUT OCCURED WHILE WAITING FOR GRIPPER TO FINISH MOVEMENT!");
			return false;
		}
	}
}

/**
 *  \brief Prints a message and return false if the function is not implemented for this gripper-type
 *  \return Returns false if the function is not implemented for this gripper-type
 */
bool gripper::functionNotImplementedError(){
	Serial.println("ERROR: Function not implemented for this gripper type");
	return false;
}
