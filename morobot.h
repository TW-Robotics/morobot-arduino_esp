#include <Arduino.h>
#include <MakeblockSmartServo.h>

#ifndef MOROBOT_H
#define MOROBOT_H

#define BREAK_LOOSE  1
#define BREAK_BRAKED 0
#define NUM_MAX_SERVOS 10
#define TIMEOUT_DELAY 15000

class morobotClass {
	public:
		morobotClass();
		void begin(uint8_t numSmartServos, const char* stream);
		void setZero();
		void moveHome();
		void setSpeedRPM(uint8_t speed);
		
		/* BREAKS */
		void setBreaks();
		void releaseBreaks();

		/* ROBOT STATUS */
		void setBusy();
		void setIdle();
		bool isReady();
		void waitUntilIsReady();
		bool checkIfMotorMoves(uint8_t servoId);
		
		/* GETTERS */
		long getActAngle(uint8_t servoId);
		float getSpeed(uint8_t servoId);
		float getTemp(uint8_t servoId);
		float getVoltage(uint8_t servoId);
		float getCurrent(uint8_t servoId);
		
		/* MOVEMENTS */
		void moveToAngle(uint8_t servoId, long angle);
		void moveToAngle(uint8_t servoId, long angle, uint8_t speedRPM);
		void moveToAngles(long angles[]);
		void moveToAngles(long angles[], uint8_t speedRPM);
		void moveAngle(uint8_t servoId, long angle);
		void moveAngle(uint8_t servoId, long angle, uint8_t speedRPM);
		void moveAngles(long angles[]);
		void moveAngles(long angles[], uint8_t speedRPM);
		
		/* HELPER */
		void printAngles(long angles[]);
		
		/* PUBLIC VARIABLES */
		MakeblockSmartServo smartServos;
		bool positionReached[NUM_MAX_SERVOS];
	private:
		uint8_t _numSmartServos;
		uint8_t _speedRPM;
		Stream* _port;
		//bool _positionReached[];
};

void callback_done(uint8_t servoNum);

//extern morobotClass morobot;

#endif