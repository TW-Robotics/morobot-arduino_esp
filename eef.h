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
 *  			gripper(int8_t servoPin);
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
 *  			binaryEEF(int8_t pin);
 *  			void activate();
 *  			void deactivate();
 *  			bool isActivated();
 *  			bool isDeactivated();
 */

#include <morobot.h>
#include <Servo.h>

#ifndef EEF_H
#define EEF_H

#define TIMEOUT_DELAY_GRIPPER 25000

class gripper {
	public:
		gripper(morobotClass* morobotToAttachTo);
		void begin();//Store pointer to object to control smartServo object; Set TCP of morobot when attach or setTCPoffset is called
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
		bool isOpen();
		morobotClass* morobot;
		Servo servo;
	private:
		bool checkIfAngleValid(float angle);
		bool waitUntilFinished();
		bool functionNotImplementedError();
		bool _isClosed;
		bool _isOpen;
		float _currentAngle;
		
		uint8_t _gripperType;	// 0: smartServo, 1: normal servo
		float _gearRatio;		// mm per degree 		only for parallel grippers
		bool _closingDirectionIsPositive;		
		uint8_t _speed[2];		// opening, closing
		float _degOpen;
		float _degClosed;
		float _degOpenLimit;
		float _degCloseLimit;
		float _closingWidthOffset;	// If gripper is completely closed, it has this gripping width
		float _tcpOffset[3];	// Position of the TCP (tool center point) with respect to the center of the flange of the last robot axis
		int8_t _servoID;
		int8_t _servoPin;
};

class binaryEEF {
	public:
		binaryEEF(int8_t pin);
		void activate();
		void deactivate();
		bool isActivated();
		bool isDeactivated();
	private:
		int8_t _pin;
};

#endif