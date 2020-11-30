/**
 *  \class 	morobotScaraRRP
 *  \brief 	morobot child class for Scara-RRP-Robot for microcontrollers such as Arduino or ESP32
 *  @file 	morobotScaraRRP.cpp
 *  @author	Johannes Rauer FHTW
 *  @date	2020/11/27
 *  \par Method List:
 *  	public:
 *  		morobotScaraRRP() : morobotClass(3){};
			virtual void setTCPoffset(float xOffset, float yOffset, float zOffset);
			virtual bool checkIfAngleValid(uint8_t servoId, float angle);
			bool checkIfAnglesValid(float phi1, float phi2, float phi3);
		protected:
			virtual bool calculateAngles(float x, float y, float z);
			virtual void updateCurrentXYZ();
 */
 
#include <morobotScaraRRP.h>

/**
 *  \brief Set the position of the TCP (tool center point) with respect to the center of the flange of the last robot axis.
 *  \param [in] xOffset Offset in x-direction
 *  \param [in] yOffset Offset in y-direction
 *  \param [in] zOffset Offset in z-direction
 *  \details This information is necessary to calculate the inverse kinematics correctly.
 *  		 The function stores the TCP-Offset and recalculates the length and angle of the last axis.
 *  		 Internally the length and angle of the last axis is stored as if it would be a straigth axis directly to the TCP.
 */
void morobotScaraRRP::setTCPoffset(float xOffset, float yOffset, float zOffset){
	_tcpOffset[0] = xOffset;
	_tcpOffset[1] = yOffset;
	_tcpOffset[2] = zOffset;
	
	// Calculate new length and angle of last axis (since eef is connected to it statically)
    c_new = sqrt(pow(_tcpOffset[1],2) + pow(c+_tcpOffset[0],2));
    beta = asin(fabs(_tcpOffset[1]/c_new));
	
	// Precalculate squares of lengths for faster processing
	c_newSQ = pow(c_new,2);
	bSQ = pow(b,2);
}

/**
 *  \brief Checks if a given angle can be reached by the joint. Each joint has a specific limit to protect the robot's mechanics.
 *  
 *  \param [in] servoId Number of motor to move (first motor has ID 0)
 *  \param [in] angle Angle to move the robot to in degrees
 *  \return Returns true if the position is reachable; false if it is not.
 */
bool morobotScaraRRP::checkIfAngleValid(uint8_t servoId, float angle){
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
bool morobotScaraRRP::checkIfAnglesValid(float phi1, float phi2, float phi3){
	float angles[3] = {phi1, phi2, phi3};
	
	for (uint8_t i = 0; i < _numSmartServos; i++) if(checkIfAngleValid(i, angles[i]) == false) return false;
	return true;
}

/**
 *  \brief Moves all motors to desired angles (Moves the whole robot) - absolute movement.
 *  \param [in] phi1, phi2, phi3 Desired angles of all motors.
 *  \details Waits until the robot is ready to use (no motor moves) before moving.
 *  		 Checks if the angles are valid before moving.
 */
void morobotScaraRRP::moveToAngles(long phi1, long phi2, long phi3){
	long angles[3] = {phi1, phi2, phi3};
	moveToAngles(angles);
}

/**
 *  \brief Calibrates the linear axis by increasing the angle until a current limit is reached
 *  \param [in] maxMotorCurrent (Optional) Current limit at which zero position is reached an calibration stops
 */
void morobotScaraRRP::moveZAxisIn(uint8_t maxMotorCurrent){
	autoCalibrateLinearAxis(2, maxMotorCurrent);
	waitUntilIsReady();
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
bool morobotScaraRRP::calculateAngles(float x, float y, float z){
    x = x-a;	// Base is in x-orientation --> Just subtract its length from x-coordinate
	z = z-_tcpOffset[2];
	
	float xSQ = pow(x,2);
	float ySQ = pow(y,2);
	
	// Calculate angle for 2nd axis
	float phi2 = acos((xSQ + ySQ - bSQ - c_newSQ) / (2*b*c_new));
	phi2 = (phi2 - beta) * 180/M_PI;
	
	// Calculate angle for 1st axis
	float gamma = atan2(y, x);
	float phi1 = acos((xSQ + ySQ + bSQ - c_newSQ) / (2*b*sqrt(xSQ + ySQ)));
	phi1 = gamma + phi1;
    phi1 = phi1 * 180/M_PI;
	
	// Calculate angle for 3rd axis (z-direction)
	float phi3 = z * gearRatio;
	
	// Check if angles are valid
	if (!checkIfAnglesValid(phi1, phi2, phi3)) return false;
	
	_goalAngles[0] = phi1;
	_goalAngles[1] = phi2;
	_goalAngles[2] = phi3;
	
	return true;
}

/**
 *  \brief Re-calculates the internally stored robot TCP position (Solves forward kinematics).
 *  \details This function does calculate and store the TCP position depending on the current motor angles.
 */
void morobotScaraRRP::updateCurrentXYZ(){
	setBusy();
	waitUntilIsReady();
	
	long actAngles[_numSmartServos];
	for (uint8_t i=0; i<_numSmartServos; i++) actAngles[i] = getActAngle(i);
	
	float xnb = b*cos((float)actAngles[0]*M_PI/180);
    float ynb = b*sin((float)actAngles[0]*M_PI/180);
    float delta = asin(xnb/b);
    float phi2s = M_PI-delta-M_PI/2-((float)actAngles[1]*M_PI/180+beta);
    float xncn = c_new*cos(phi2s);
    float yncn = c_new*sin(phi2s);
    
	_actPos[0] = a + xnb + xncn;
    _actPos[1] = ynb + yncn;
	_actPos[2] = actAngles[2]/gearRatio + _tcpOffset[2];
	
	Serial.print("Calculated Position: ");
	Serial.print(_actPos[0]);
	Serial.print(", ");
	Serial.print(_actPos[1]);
	Serial.print(", ");
	Serial.println(_actPos[2]);
}