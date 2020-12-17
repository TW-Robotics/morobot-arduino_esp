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

#include <morobot.h>

#ifndef EEF_H
#define EEF_H

class gripper {
	public:
		gripper();
		gripper(uint8_t servoPin);
		void attach(morobotClass* morobot);	//Store pointer to object to control smartServo object; Set TCP of morobot when attach or setTCPoffset is called
		void setParams(float degOpen, float degClosed, float degLimitMin, float degLimitMax);
		void setTCPoffset(float xOffset, float yOffset, float zOffset);
		void close();
		void open();
		bool moveToWidth(float width);
		bool moveToAngle(float angle);
		bool isClosed();
		bool isOpen();
		morobotClass* morobot;
	private:
		float _degOpen;
		float _degClosed;
		float _degLimits[2];
		float _tcpOffset[3];	// Position of the TCP (tool center point) with respect to the center of the flange of the last robot axis
		uint8_t _servoID;
		uint8_t _servoPin;
};

class binaryEEF {
	public:
		binaryEEF(uint8_t pin);
		void activate();
		void deactivate();
		bool isActivated();
		bool isDeactivated();
	private:
		uint8_t _pin;
};

#endif