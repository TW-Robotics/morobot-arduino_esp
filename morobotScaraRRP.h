#include <Arduino.h>
#include <MakeblockSmartServo.h>
#include <morobot.h>

#ifndef MOROBOTSCARARRP_H
#define MOROBOTSCARARRP_H

class morobotScaraRRP:public morobotClass {
	public:

		/* ROBOT STATUS */
		virtual void updateCurrentXYZ();
		
		virtual void setTCPpos(float xOffset, float yOffset, float zOffset);
		
		/* HELPER */
		virtual bool calculateAngles(float x, float y, float z);
	private:
			// TODO: Change for other robots
		float a = 47.0;		// From mounting to first axis
		float b = 92.9;		// From first axis to second axis
		float c = 72.79;	// From second axis to center of flange
		float gearRatio = 16.25;		// Turn motor of linear axis by gearRatio degrees to move it 1 mm
		float c_new;
		float beta;
		float c_newSQ;
		float bSQ;

};

#endif