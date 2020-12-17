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
			bool checkIfAnglesValid(float phi1, float phi2, float phi3);
		protected:
			virtual bool calculateAngles(float x, float y, float rotZ);
			virtual void updateCurrentXYZ();
 */
 
#include <morobotScaraRRR.h>

/**
 *  \brief Set the position of the TCP (tool center point) with respect to the center of the flange of the last robot axis.
 *  \param [in] xOffset Offset in x-direction
 *  \param [in] yOffset Offset in y-direction
 *  \param [in] zOffset Offset in z-direction
 *  \details This information is necessary to calculate the inverse kinematics correctly.
 *  		 The function stores the TCP-Offset and recalculates the length and angle of the last axis.
 *  		 Internally the length and angle of the last axis is stored as if it would be a straigth axis directly to the TCP.
 */
void morobotScaraRRR::setTCPoffset(float xOffset, float yOffset, float zOffset){
	_tcpOffset[0] = xOffset;
	_tcpOffset[1] = yOffset;
	_tcpOffset[2] = zOffset;
	
	// Calculate new length and angle of last axis (since eef is connected to it statically)
	//TODO: Add code for y-offset
	d = xOffset;
	dSQ = pow(d,2);
	bSQ = pow(b,2);
	cSQ = pow(c,2);
	
	updateTCPpose();
}

/**
 *  \brief Checks if a given angle can be reached by the joint. Each joint has a specific limit to protect the robot's mechanics.
 *  
 *  \param [in] servoId Number of motor to move (first motor has ID 0)
 *  \param [in] angle Angle to move the robot to in degrees
 *  \return Returns true if the position is reachable; false if it is not.
 */
bool morobotScaraRRR::checkIfAngleValid(uint8_t servoId, float angle){
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

/**
 *  \brief Checks if all robot motor angles are valid
 *  \param [in] phi1 Angle value for first joint
 *  \param [in] phi2 Angle value for second joint
 *  \param [in] phi3 Angle value for third joint
 *  \return Returns true if the position is reachable; false if it is not.
 */
bool morobotScaraRRR::checkIfAnglesValid(float phi1, float phi2, float phi3){
	float angles[3] = {phi1, phi2, phi3};
	
	for (uint8_t i = 0; i < _numSmartServos; i++) if(checkIfAngleValid(i, angles[i]) == false) return false;
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
bool morobotScaraRRR::calculateAngles(float x, float y, float rotZ){
	rotZ = rotZ * M_PI/180;			// Transform rotation into radians
	x = x-a;							// Base is in x-orientation --> Just subtract base-length from x-coordinate
	
	// Calculate position for center of rotation of last axis
	float x_w = x - d*cos(rotZ);
	float y_w = y - d*sin(rotZ);
	float x_wSQ = pow(x_w, 2);
	float y_wSQ = pow(y_w, 2);
	
	float phi2 = acos((x_wSQ + y_wSQ - bSQ - cSQ)/(2 * b * c));
	float gamma = acos((x_wSQ + y_wSQ + bSQ - cSQ)/(2 * b * sqrt(x_wSQ + y_wSQ)));
	float alpha = atan2(y_w, x_w);
	float phi1 = - (alpha - gamma);
	float phi3 = - (rotZ - (phi2 - phi1));
	
	phi1 = convertToDegrees(phi1);
	phi2 = convertToDegrees(phi2);
	phi3 = convertToDegrees(phi3);
	
	// Check if angles are valid
	if (!checkIfAnglesValid(phi1, phi2, phi3)){
		// Try out redundant configuration
		Serial.println("Switching to other configuration");
		phi1 = - (-phi1 + 2*gamma);
		phi3 = - (-phi3 + 2*(phi2-gamma));
		phi2 = - phi2;
		
		phi1 = convertToDegrees(phi1);
		phi2 = convertToDegrees(phi2);
		phi3 = convertToDegrees(phi3);
		if (!checkIfAnglesValid(phi1, phi2, phi3)) return false;
	}
	_goalAngles[0] = phi1;		// Since motor 0 is installed in other direction
	_goalAngles[1] = phi2;
	_goalAngles[2] = phi3;
	
	return true;
}

/**
 *  \brief Re-calculates the internally stored robot TCP position (Solves forward kinematics).
 *  \details This function does calculate and store the TCP position depending on the current motor angles.
 */
void morobotScaraRRR::updateTCPpose(){
	setBusy();
	waitUntilIsReady();
	
	// Get anlges of all motors
	long actAngles[_numSmartServos];
	for (uint8_t i=0; i<_numSmartServos; i++) actAngles[i] = getActAngle(i);
	
	// Calculate lengths at each joint and sum up
	float xnb = b*cos(-(float)actAngles[0]*M_PI/180);
    float ynb = b*sin(-(float)actAngles[0]*M_PI/180);
    float xnc = c*cos(-(float)actAngles[0]*M_PI/180 + (float)actAngles[1]*M_PI/180);
    float ync = c*sin(-(float)actAngles[0]*M_PI/180 + (float)actAngles[1]*M_PI/180);
    float xnd = d*cos(-(float)actAngles[0]*M_PI/180 + (float)actAngles[1]*M_PI/180 - (float)actAngles[2]*M_PI/180);
    float ynd = d*sin(-(float)actAngles[0]*M_PI/180 + (float)actAngles[1]*M_PI/180 - (float)actAngles[2]*M_PI/180);
    
	_actPos[0] = a + xnb + xnc + xnd;
    _actPos[1] = ynb + ync + ynd;
	_actPos[2] = _tcpOffset[2];
	
	// Caculate orientation
	_actOri[0] = 0;
	_actOri[1] = 0;
	_actOri[2] = -actAngles[0] + actAngles[1] - actAngles[2];
	
	Serial.print("Calculated Position: ");
	Serial.print(_actPos[0]);
	Serial.print(", ");
	Serial.print(_actPos[1]);
	Serial.print(", ");
	Serial.print(_actPos[2]);
	Serial.print("; Orientation around z-axis [degrees]: ");
	Serial.println(_actOri[2]);
}