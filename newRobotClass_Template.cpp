/**
 *  \class 	newRobotClass
 *  \brief 	morobot child class for ADD ROBOT TYPE for microcontrollers such as Arduino or ESP32
 *  @file 	newRobotClass.h
 *  @author	Johannes Rauer FHTW
 *  @date	2020/11/27
 *  \par Method List:
 *  	public:
 *  		newRobotClass() : morobotClass(PUT_NUM_SERVOS_HERE){};
			virtual void setTCPoffset(float xOffset, float yOffset, float zOffset);
			virtual bool checkIfAngleValid(uint8_t servoId, float angle);
		protected:
			virtual bool calculateAngles(float x, float y, float z);
			virtual void updateCurrentXYZ();
 */
 
#include <newRobotClass_Template.h>

/**
 *  \brief Set the position of the TCP (tool center point) with respect to the center of the flange of the last robot axis.
 *  \param [in] xOffset Offset in x-direction
 *  \param [in] yOffset Offset in y-direction
 *  \param [in] zOffset Offset in z-direction
 *  \details This information is necessary to calculate the inverse kinematics correctly.
 *  		 The function stores the TCP-Offset and recalculates the length and angle of the last axis.
 *  		 Internally the length and angle of the last axis is stored as if it would be a straigth axis directly to the TCP.
 */
void newRobotClass_Template::setTCPoffset(float xOffset, float yOffset, float zOffset){
	_tcpOffset[0] = xOffset;
	_tcpOffset[1] = yOffset;
	_tcpOffset[2] = zOffset;
	
	// Calculate new length and angle of last axis (since eef is connected to it statically)
	//TODO: PUT YOUR CODE TO RE-CALCULATE LENGTHS OF ROBOT HERE 
}

/**
 *  \brief Checks if a given angle can be reached by the joint. Each joint has a specific limit to protect the robot's mechanics.
 *  
 *  \param [in] servoId Number of motor to move (first motor has ID 0)
 *  \param [in] angle Angle to move the robot to in degrees
 *  \return Returns true if the position is reachable; false if it is not.
 */
bool newRobotClass_Template::checkIfAngleValid(uint8_t servoId, float angle){
	// The values are NAN if the inverse kinematics does not provide a solution
	if(isnan(angle)){
		Serial.print("Angle for motor ");
		Serial.print(servoId);
		Serial.println(" is NAN!");
		return false;
	}
	
	// Moving the motors out of the joint limits may harm the robot's mechanics
	if(angle < _jointLimits[servoId][0] || angle > _jointLimits[servoId][1]) {
		Serial.print("Angle for motor ");
		Serial.print(servoId);
		Serial.print(" is invalid! (");
		Serial.print(angle);
		Serial.println(" degrees).");
		return false;
	}
	return true;
}


/* PROTECTED FUNCTIONS */
/**
 *  \brief Uses given coordinates to calculate the motor angles to reach this position (Solve inverse kinematics).
 *  \param [in] x Desired x-position of TCP
 *  \param [in] y Desired x-position of TCP
 *  \param [in] z Desired x-position of TCP
 *  \return Returns true if the position is reachable; false if it is not.
 *  \details This function does only calculate the angles of the motors and stores them internally.
 *  		 Use moveToPosition(x,y,z) to actually move the robot.
 */
bool newRobotClass_Template::calculateAngles(float x, float y, float z){
	//TODO: CALCULATE THE MOTOR VALUES USING THE INVERSE KINEMATICS
	
	//TODO: CHECK IF THE ANGLES ARE VALID E.G.:
	//if (!checkIfAnglesValid(phi1, phi2, phi3)) return false;
	
	//TODO: IF THE ANGLES ARE VALID: STORE YOUR CALCULATED ANGLES E.G.:
	//_goalAngles[0] = phi1;
	
	return true;
}

/**
 *  \brief Re-calculates the internally stored robot TCP position (Solves forward kinematics).
 *  \details This function does calculate and store the TCP position depending on the current motor angles.
 */
void newRobotClass_Template::updateCurrentXYZ(){
	setBusy();
	waitUntilIsReady();
	
	//TODO: SOLVE FORWARD KINEMATICS TO GET TCP POSITION
	
	//TODO: STORE THE POSITION FOR ALL AXES E.G.:
	//_actPos[0] = a + xnb + xncn;
	
	Serial.print("Calculated Position: ");
	Serial.println(_actPos[0]);
	Serial.println(_actPos[1]);
	Serial.println(_actPos[2]);
}