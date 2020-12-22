/**
 *  \class 	morobotScaraRRR
 *  \brief 	morobot child class for ADD ROBOT TYPE for microcontrollers such as Arduino or ESP32
 *  @file 	morobotScaraRRR.h
 *  @author	Johannes Rauer FHTW
 *  @date	2020/11/27
 *  \par Method List:
 *  	public:
 *  		morobotScaraRRR() : morobotClass(PUT_NUM_SERVOS_HERE){};
			virtual void setTCPoffset(float xOffset, float yOffset, float zOffset);
			virtual bool checkIfAngleValid(uint8_t servoId, float angle);
			bool checkIfAnglesValid(float phi1, float phi2, float phi3);
		protected:
			virtual bool calculateAngles(float x, float y, float rotZ);
			virtual void updateCurrentXYZ();
 */

#ifndef MOROBOTSCARARRP_H
#define MOROBOTSCARARRP_H

#include <morobot.h>

class morobotScaraRRR:public morobotClass {
	public:
		/**
		 *  \brief Constructor of morobotScaraRRR class
		 *  \details The value in brakets defines that the robot consists of three smartservos
		 */
		morobotScaraRRR() : morobotClass(3){};
		
		/**
		 *  \brief Set the position of the TCP (tool center point) with respect to the center of the flange of the last robot axis.
		 * 			 This information is necessary to calculate the inverse kinematics correctly.
		 *  		 The function stores the TCP-Offset and recalculates the length and angle of the last axis.
		 *  		 Internally the length and angle of the last axis is stored as if it would be a straigth axis directly to the TCP.
		 *  \param [in] xOffset Offset in x-direction
		 *  \param [in] yOffset Offset in y-direction
		 *  \param [in] zOffset Offset in z-direction
		 */
		virtual void setTCPoffset(float xOffset, float yOffset, float zOffset);
		
		/**
		 *  \brief Checks if a given angle can be reached by the joint. Each joint has a specific limit to protect the robot's mechanics.
		 *  		The joint limits are predefined in the private variable _jointLimits
		 *  \param [in] servoId Number of motor to move (first motor has ID 0)
		 *  \param [in] angle Angle to move the robot to in degrees
		 *  \return Returns true if the position is reachable; false if it is not.
		 */
		virtual bool checkIfAngleValid(uint8_t servoId, float angle);
		
		/**
		 *  \brief Checks if all robot motor angles are valid. Just calls checkIfAngleValid for each motor times.
		 *  \param [in] phi1 Angle value for first joint
		 *  \param [in] phi2 Angle value for second joint
		 *  \param [in] phi3 Angle value for third joint
		 *  \return Returns true if the position is reachable; false if it is not.
		 */
		bool checkIfAnglesValid(float phi1, float phi2, float phi3);

	protected:
		/**
		 *  \brief Uses given coordinates to calculate the motor angles to reach this position (Solve inverse kinematics).
		 *  		This function does only calculate the angles of the motors and stores them internally.
		 *  		Use moveToPosition(x,y,z) to actually move the robot.
		 *  \param [in] x Desired x-position of TCP
		 *  \param [in] y Desired y-position of TCP
		 *  \param [in] rotZ Desired rotation of TCP around z-axis
		 *  \return Returns true if the position is reachable; false if it is not.
		 */
		virtual bool calculateAngles(float x, float y, float rotZ);
		
		/**
		 *  \brief Re-calculates the internally stored robot TCP position (Solves forward kinematics).
		 *  		This function does calculate and store the TCP position depending on the current motor angles.
		 *  \param [in] output If output = true, the calculated position+orientation is printed to the terminal
		 */
		virtual void updateTCPpose(bool output = false);

	private:
		float _tcpOffset[3];		//!< Position of the TCP (tool center point) with respect to the center of the flange of the last robot axis
		long _jointLimits[3][2] = {{-100, 100}, {-100, 100}, {-180, 180}};		//!< Limits for all joints
		
		float a = 47.0;				//!< Length from mounting to first axis
		float b = 92.9;				//!< Length from first axis to second axis
		float c = 70.52;			//!< Lenth from second axis to third axis
		float d = 0;				//!< Length from third axis to TCP (set by setTCPOffset)
		float bSQ;					//!< Square of b (Precalculated for faster processing)
		float cSQ;					//!< Square of c (Precalculated for faster processing)
		float dSQ;					//!< Square of d (Precalculated for faster processing)
};

#endif