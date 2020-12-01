/**
 *  \class 	morobotClass
 *  \brief 	morobot base class for microcontrollers such as Arduino or ESP32
 *  @file 	morobot.cpp
 *  @author	Johannes Rauer FHTW
 *  @date	2020/11/27
 *  \par Method List:
 *  	public:
 *  		morobotClass(uint8_t numSmartServos);
			void begin(const char* stream);
			void setZero();
			void moveHome();
			void setSpeedRPM(uint8_t speed);
			virtual void setTCPoffset(float xOffset, float yOffset, float zOffset);
			virtual bool checkIfAngleValid(uint8_t servoId, float angle);
			
			void setBreaks();
			void releaseBreaks();

			void setBusy();
			void setIdle();
			void waitUntilIsReady();
			bool checkIfMotorMoves(uint8_t servoId);
			
			long getActAngle(uint8_t servoId);
			float getActPosition(char axis);
			float getSpeed(uint8_t servoId);
			float getTemp(uint8_t servoId);
			float getVoltage(uint8_t servoId);
			float getCurrent(uint8_t servoId);
			
			void moveToAngle(uint8_t servoId, long angle);
			void moveToAngle(uint8_t servoId, long angle, uint8_t speedRPM, bool checkValidity=true);
			void moveToAngles(long angles[]);
			void moveToAngles(long angles[], uint8_t speedRPM);
			void moveAngle(uint8_t servoId, long angle);
			void moveAngle(uint8_t servoId, long angle, uint8_t speedRPM, bool checkValidity=true);
			void moveAngles(long angles[]);
			void moveAngles(long angles[], uint8_t speedRPM);
			bool moveToPosition(float x, float y, float z);
			bool moveXYZ(float xOffset, float yOffset, float zOffset);
			bool moveInDirection(char axis, float value);
			
			void printAngles(long angles[]);
		protected:
			virtual bool calculateAngles(float x, float y, float z);
			virtual void updateTCPpose();
			void autoCalibrateLinearAxis(uint8_t servoId, uint8_t maxMotorCurrent=25);
		private:
			bool isReady();
 */

#include <morobot.h>

//TODO: Document Kinematics
//TODO: Implement second robot for ESP32

/**
 *  \brief Constructor of morobot class
 *  \param [in] numSmartServos Number of smart servos of the robot
 */
morobotClass::morobotClass(uint8_t numSmartServos){
	if (numSmartServos > NUM_MAX_SERVOS){
		Serial.print("Too many motors! Maximum number of motors: ");
		Serial.println(NUM_MAX_SERVOS);
	}
	_numSmartServos = numSmartServos;
}

/**
 *  \brief Starts the communication with the smartservos of the robot
 *  
 *  \param [in] stream Name of serial port (e.g. "Serial1").
 *  \return None
 *  
 *  \details The names of the Serial ports for Arduino controllers can be found here: https://www.arduino.cc/reference/en/language/functions/communication/serial/
 */
void morobotClass::begin(const char* stream){
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
	
    smartServos.beginSerial(_port);
	delay(5);
    smartServos.assignDevIdRequest();
    delay(50);
	
	setTCPoffset(0, 0, 0);
	setSpeedRPM(25);
	updateTCPpose();

	Serial.println("Connected to motors");
}

/**
 *  \brief Sets the current position as origin (zero position)
 *  \details Call this function after bringing the motors into their initial (zero position) to store it permanently as 0 degrees
 */
void morobotClass::setZero(){
	for (uint8_t i=0; i<_numSmartServos; i++) smartServos.setZero(i+1);
}

/**
 *  \brief Moves all motors to their origin (zero position)
 *  \details Only call this function after calibration (Setting a zero position for all motors using setZero())
 */
void morobotClass::moveHome(){
	for (uint8_t i=0; i<_numSmartServos; i++) smartServos.setInitAngle(i+1, 0, 15);
	waitUntilIsReady();
}

/**
 *  \brief Set the speed in RPM (rounds per minute) for all motors
 *  \param [in] speed Desired velocity of the motors in RPM (rounds per minute). Values accepted between 1 and 50.
 *  \details Use this function to set a default speed for all motors so you do not have to give a speed every time the robot should move.
 *  		 If a different speed is given in a movement-function, this speed will be used only for this movement.
 */
void morobotClass::setSpeedRPM(uint8_t speed){
	_speedRPM = speed;
	
	// If the speed is bigger than the maximum speed, set it to maximum.
	// If the speed is smaller than then 1, set it to minimum.
	if (speed > SERVO_MAX_SPEED_RPM) _speedRPM = SERVO_MAX_SPEED_RPM;
	if (speed < 0) _speedRPM = 1;
}


/* BREAKS */
/**
 *  \brief Sets the breaks of all motors. Axes cannot be moved after calling this function.
 *  \details After moving a motor with a movement-function, the motor is breaked at final position automatically. 
 */
void morobotClass::setBreaks(){
	for (uint8_t i=0; i<_numSmartServos; i++) smartServos.setBreak(i+1, BREAK_BRAKED);
}

/**
 *  \brief Releases the breaks of all motors. Axes can be moved freely after calling this function.
 *  \details After moving a motor with a movement-function, the motor is breaked at final position automatically.
 *  		 Use this function to release breaks to move the motor with your hands.
 */
void morobotClass::releaseBreaks(){
	for (uint8_t i=0; i<_numSmartServos; i++) smartServos.setBreak(i+1, BREAK_LOOSE);
}


/* ROBOT STATUS */
/**
 *  \brief Sets internal variables which indicate, that the robot should not be moved at the moment.
 *  \details Use setIdle() to indicate, that the robot can be moved.
 *  		 Use waitUntilIsReady() to wait until the robot is idle (no motor moves) before continuing.
 */
void morobotClass::setBusy(){
	for (uint8_t i=0; i<_numSmartServos; i++) _angleReached[i] = false;
}

/**
 *  \brief Sets internal variables which indicate, that the robot can be moved.
 *  \details Use setBusy() to indicate, that the robot should not be moved.
 */
void morobotClass::setIdle(){
	for (uint8_t i=0; i<_numSmartServos; i++) _angleReached[i] = true;
}

/**
 *  \brief Waits until the robot is ready for new commants (all motors have stopped moving) or a timeout occurs.
 *  \details Function sets the robot idle only when all motors have stopped moving or a timeout occurs.
 *  		 If a timeout occurs this is printed to the serial monitor.
 */
void morobotClass::waitUntilIsReady(){
	setBusy();
	unsigned long startTime = millis();
	while (true){
		// Check if the robot is ready yet
		if (isReady() == true) break;
		// Stop waiting if the robot is not ready after a timeout occurs
		if ((millis() - startTime) > TIMEOUT_DELAY) {
			Serial.println("TIMEOUT OCCURED WHILE WAITING FOR ROBOT TO FINISH MOVEMENT!");
			break;
		}
	}
	setIdle();
}
		
/**
 *  \brief Check if a given smart servo is moving at the moment.
 *  \param [in] servoId Number of motor to check (first motor has ID 0)
 *  \return Returns True if motor is moving; False if motor is not moving.
 *  \details Function stores current angle of motor, waits some time and compares the angle before and after waiting.
 */
bool morobotClass::checkIfMotorMoves(uint8_t servoId){
	long startPos = getActAngle(servoId);
	delay(100);
	if (startPos != getActAngle(servoId)) return true;
	return false;
}


/* GETTERS */
/**
 *  \brief Returns angle-position of motor in degrees.
 *  \param [in] servoId Number of motor (first motor has ID 0)
 *  \return Angle-positon in degrees.
 */
long morobotClass::getActAngle(uint8_t servoId){
	return smartServos.getAngleRequest(servoId+1);
}

/**
 *  \brief Returns position of TCP in mm in given axis (in robot base frame).
 *  \param [in] axis Possible parameters: 'x', 'y', 'z'
 *  \return Position of TCP in mm in given axis.
 */
float morobotClass::getActPosition(char axis){
	updateTCPpose();
	
	if (axis == 'x') return _actPos[0];
	else if (axis == 'y') return _actPos[1];
	else if (axis == 'z') return _actPos[2];
	else Serial.println("ERROR! Invalid axis in getActPosition();");
}

/**
 *  \brief Returns current speed of motor in RPM (rounds per minute).
 *  \param [in] servoId Number of motor (first motor has ID 0)
 *  \return Current speed of motor in RPM.
 */
float morobotClass::getSpeed(uint8_t servoId){
	return smartServos.getSpeedRequest(servoId+1);
}

/**
 *  \brief Returns current temperature of motor in degrees Celsius.
 *  \param [in] servoId Number of motor (first motor has ID 0)
 *  \return Current temperature of motor in degrees Celsius.
 */
float morobotClass::getTemp(uint8_t servoId){
	return smartServos.getTempRequest(servoId+1);
}

/**
 *  \brief Returns current voltage of motor.
 *  \param [in] servoId Number of motor (first motor has ID 0)
 *  \return Current voltage of motor.
 */
float morobotClass::getVoltage(uint8_t servoId){
	return smartServos.getVoltageRequest(servoId+1);
}

/**
 *  \brief Returns current current consumption of motor in Ampere.
 *  \param [in] servoId Number of motor (first motor has ID 0)
 *  \return Current current consumption of motor in Ampere.
 */
float morobotClass::getCurrent(uint8_t servoId){
	return smartServos.getCurrentRequest(servoId+1);
}


/* MOVEMENTS */
/**
 *  \brief Moves a motor to a desired angle (absolute movement).
 *  \param [in] servoId Number of motor (first motor has ID 0)
 *  \param [in] angle Desired goal angle in degrees.
 *  \param [in] speedRPM (Optional) Desired velocity of the motor in RPM (rounds per minute). Values accepted between 1 and 50. If no speed is given, the preset default speed is used.
 *  \param [in] checkValidity (Optional) Set this variable to "false" if you don't want to check if the angle value is valid (e.g. necessary for calibration)
 *  \details Checks if the angle is valid before moving if checkValidity is not given or true.
 */
void morobotClass::moveToAngle(uint8_t servoId, long angle){
	if (checkIfAngleValid(servoId, angle) == true) smartServos.moveTo(servoId+1, angle, _speedRPM);
}

void morobotClass::moveToAngle(uint8_t servoId, long angle, uint8_t speedRPM, bool checkValidity){
	if (checkValidity == false) smartServos.moveTo(servoId+1, angle, speedRPM);
	else if (checkIfAngleValid(servoId, angle) == true) smartServos.moveTo(servoId+1, angle, speedRPM);
}

/**
 *  \brief Moves all motors to desired angles (Moves the whole robot) - absolute movement.
 *  \param [in] angles[] Desired goal angles in degrees.
 *  \param [in] speedRPM Optional) Desired velocity of the motor in RPM (rounds per minute). Values accepted between 1 and 50. If no speed is given, the preset default speed is used.
 *  \details Waits until the robot is ready to use (no motor moves) before moving.
 *  		 Checks if the angles are valid before moving.
 */
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

/**
 *  \brief Moves a motor by a desired angle (relative movement).
 *  \param [in] servoId Number of motor (first motor has ID 0)
 *  \param [in] angle Desired goal angle in degrees.
 *  \param [in] speedRPM (Optional) Desired velocity of the motor in RPM (rounds per minute). Values accepted between 1 and 50. If no speed is given, the preset default speed is used.
 *  \param [in] checkValidity (Optional) Set this variable to "false" if you don't want to check if the angle value is valid (e.g. necessary for calibration)
 *  \details Checks if the angle is valid before moving if checkValidity is not given or true.
 */
void morobotClass::moveAngle(uint8_t servoId, long angle){
	if (checkIfAngleValid(servoId, getActAngle(servoId)+angle) == true) smartServos.move(servoId+1, angle, _speedRPM);
}

void morobotClass::moveAngle(uint8_t servoId, long angle, uint8_t speedRPM, bool checkValidity){
	if (checkValidity == false) smartServos.move(servoId+1, angle, speedRPM);
	else if (checkIfAngleValid(servoId, getActAngle(servoId)+angle) == true) smartServos.move(servoId+1, angle, speedRPM);
}

/**
 *  \brief Moves all motors by desired angles (Moves the whole robot) - relative movement.
 *  \param [in] angles[] Desired angles in degrees to move robot by.
 *  \param [in] speedRPM (Optional) Desired velocity of the motor in RPM (rounds per minute). Values accepted between 1 and 50. If no speed is given, the preset default speed is used.
 *  \details Waits until the robot is ready to use (no motor moves) before moving.
 *  		 Checks if the angles are valid before moving.
 */
void morobotClass::moveAngles(long angles[]){
	waitUntilIsReady();
	setBusy();
	Serial.print("Moving [deg]: ");
	printAngles(angles);

	for (uint8_t i=0; i<_numSmartServos; i++) moveAngle(i, angles[i]);
}

void morobotClass::moveAngles(long angles[], uint8_t speedRPM){
	waitUntilIsReady();
	setBusy();
	Serial.print("Moving [deg]: ");
	printAngles(angles);

	for (uint8_t i=0; i<_numSmartServos; i++) moveAngle(i, angles[i], speedRPM);
}

/**
 *  \brief Moves the TCP (tool center point) of the robot to a desired position.
 *  \param [in] x Desired x-coordinate of the TCP (in base frame)
 *  \param [in] y Desired y-coordinate of the TCP (in base frame)
 *  \param [in] z Desired z-coordinate of the TCP (in base frame)
 *  \return Returns true if the position is reachable; false if it is not.
 *  \details Calls child class to solve inverse kinematics and moves the robot to the position.
 */
bool morobotClass::moveToPosition(float x, float y, float z){
	Serial.print("Moving to [mm]: ");
	Serial.print(x);
	Serial.print(", ");
	Serial.print(y);
	Serial.print(", ");
	Serial.println(z);
	if (calculateAngles(x, y, z) == false) return false;
	
	for (uint8_t i=0; i<_numSmartServos; i++) moveToAngle(i, _goalAngles[i]);
	return true;
}

/**
 *  \brief Moves the TCP (tool center point) of the robot by given axis-values.
 *  \param [in] xOffset Desired x-value to move the TCP by in mm
 *  \param [in] yOffset Desired y-value to move the TCP by in mm
 *  \param [in] zOffset Desired z-value to move the TCP by in mm
 *  \return Returns true if the position is reachable; false if it is not.
 *  \details Calls child class to solve forward kinematics, adds values, solves inverse kinematics and moves the robot to the position.
 */
bool morobotClass::moveXYZ(float xOffset, float yOffset, float zOffset){
	updateTCPpose();
	return moveToPosition(_actPos[0]+xOffset, _actPos[1]+yOffset, _actPos[2]+zOffset);
}

/**
 *  \brief Moves the TCP (tool center point) of the robot by given value in one axis.
 *  \param [in] axis Axis to move the robot in. Possible parameters: 'x', 'y', 'z'
 *  \param [in] value Value by which the robot should be moved in mm
 *  \return Returns true if the position is reachable; false if it is not.
 *  \details Calls child class to solve forward kinematics, adds values, solves inverse kinematics and moves the robot to the position.
 */
bool morobotClass::moveInDirection(char axis, float value){
	updateTCPpose();
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
/**
 *  \brief Prints an array of angles to the servial monitor.
 *  \param [in] angles[] Angle values to print.
 */
void morobotClass::printAngles(long angles[]){
	for (uint8_t i=0; i<_numSmartServos; i++) {
		Serial.print(angles[i]);
		if (i != _numSmartServos-1) Serial.print(", ");
		else Serial.println(" ");
	}
}

/* PROTECTED */
void morobotClass::autoCalibrateLinearAxis(uint8_t servoId, uint8_t maxMotorCurrent){
	while(true){
		moveAngle(servoId, -2, 1, false);
		if (getCurrent(servoId) > 25) break;
	}
	smartServos.setZero(servoId+1);
	Serial.println("Linear axis set zero!");
}

/* ROBOT STATUS PRIVATE */
/**
 *  \brief Checks if the robot is busy or idle.
 *  \return Returns true if the robot is idle; false if the robot is busy
 *  \details Checks if internal variables indicate the robot is idle.
 *  		 If a motor is not already set idle, it checks if the motor is still moving.
 */
bool morobotClass::isReady(){
	for (uint8_t i=0; i<_numSmartServos; i++) {
		if (_angleReached[i] == false) {
			if (checkIfMotorMoves(i) == false) continue;
			return false;
		}
	}
	return true;
}