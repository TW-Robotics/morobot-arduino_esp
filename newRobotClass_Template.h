/**
 *  \class 	newRobotClass_Template
 *  \brief 	morobot child class for ADD ROBOT TYPE for microcontrollers such as Arduino or ESP32
 *  @file 	newRobotClass_Template.h
 *  @author	Johannes Rauer FHTW
 *  @date	2020/11/27
 *  \par Method List:
 *  	public:
 *  		newRobotClass_Template() : morobotClass(PUT_NUM_SERVOS_HERE){};
			virtual void setTCPoffset(float xOffset, float yOffset, float zOffset);
			virtual bool checkIfAngleValid(uint8_t servoId, float angle);
			bool checkIfAnglesValid(float phi1, float phi2, float phi3);
		protected:
			virtual bool calculateAngles(float x, float y, float z);
			virtual void updateCurrentXYZ();
 */

#ifndef MOROBOTSCARARRP_H
#define MOROBOTSCARARRP_H

#include <morobot.h>

class newRobotClass_Template:public morobotClass {
	public:
		/**
		 *  \brief Constructor of newRobotClass_Template class
		 *  \details The value in brakets defines that the robot consists of three smartservos
		 */
		newRobotClass_Template() : morobotClass(3){};	// TODO: PUT THE NUMBER OF SERVOS HERE
		
		/**
		 *  \brief Set the position of the TCP (tool center point) with respect to the center of the flange of the last robot axis.
		 *  \param [in] xOffset Offset in x-direction
		 *  \param [in] yOffset Offset in y-direction
		 *  \param [in] zOffset Offset in z-direction
		 *  \details This information is necessary to calculate the inverse kinematics correctly.
		 */
		virtual void setTCPoffset(float xOffset, float yOffset, float zOffset);
		
		/**
		 *  \brief Checks if a given angle can be reached by the joint. Each joint has a specific limit to protect the robot's mechanics.
		 *  
		 *  \param [in] servoId Number of motor to move (first motor has ID 0)
		 *  \param [in] angle Angle to move the robot to in degrees
		 *  \return Returns true if the position is reachable; false if it is not.
		 *  \details The joint limits are predefined in the private variable _jointLimits
		 */
		virtual bool checkIfAngleValid(uint8_t servoId, float angle);
		
		/**
		 *  \brief Checks if all robot motor angles are valid
		 *  \param [in] phi1 Angle value for first joint
		 *  \param [in] phi2 Angle value for second joint
		 *  \param [in] phi3 Angle value for third joint
		 *  \return Returns true if the position is reachable; false if it is not.
		 *  \details Just calls checkIfAngleValid for each motor times.
		 */
		bool checkIfAnglesValid(float phi1, float phi2, float phi3);

	protected:
		/**
		 *  \brief Uses given coordinates to calculate the motor angles to reach this position (Solve inverse kinematics).
		 *  \param [in] x Desired x-position of TCP
		 *  \param [in] y Desired x-position of TCP
		 *  \param [in] z Desired x-position of TCP
		 *  \return Returns true if the position is reachable; false if it is not.
		 *  \details This function does only calculate the angles of the motors and stores them internally.
		 *  		 Use moveToPosition(x,y,z) to actually move the robot.
		 */
		virtual bool calculateAngles(float x, float y, float z);
		
		/**
		 *  \brief Re-calculates the internally stored robot TCP position (Solves forward kinematics).
		 *  \details This function does calculate and store the TCP position depending on the current motor angles.
		 */
		virtual void updateTCPpose();

	private:
		float _tcpOffset[3];	// Position of the TCP (tool center point) with respect to the center of the flange of the last robot axis
		long _jointLimits[3][2] = {{-100, 100}, {-100, 100}, {0, 780}};		// Limits for all joints
				// TODO: CHANGE THE NUMBER OF SERVOS AND THE LIMITS HERE
		
		//TODO: PUT VARIABLES FOR INVERSE KINEMATICS HERE
};

#endif