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
