#include <morobotScaraRRP.h>

bool morobotScaraRRP::calculateAngles(float x, float y, float z){
    x = x-a;	// Base is in x-orientation --> Just subtract its length from x-coordinate
	z = z-_tcpPos[2];
	
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
	if (isnan(phi1) || isnan(phi2) || isnan(phi3) || fabs(phi1) > 100 || fabs(phi2) > 100 || phi3 < 0 || phi3 > 780){
		Serial.print("Position cannot be reached: ");
		Serial.print(phi1);
		Serial.print(", ");
		Serial.print(phi2);
		Serial.print(", ");
		Serial.println(phi3);
		Serial.print(x);
		Serial.print(", ");
		Serial.print(y);
		Serial.print(", ");
		Serial.println(z);
		return false;
	}
	_goalAngles[0] = phi1;
	_goalAngles[1] = phi2;
	_goalAngles[2] = phi3;
	
	/*Serial.print("Calculated Angles: ");
	Serial.print(_goalAngles[0]);
	Serial.print(", ");
	Serial.print(_goalAngles[1]);
	Serial.print(", ");
	Serial.println(_goalAngles[2]);*/
	
	return true;
}

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
	_actPos[2] = actAngles[2]/gearRatio + _tcpPos[2];
	
	Serial.print("Calculated Position: ");
	Serial.println(_actPos[0]);
	Serial.println(_actPos[1]);
	Serial.println(_actPos[2]);
}

void morobotScaraRRP::setTCPpos(float xOffset, float yOffset, float zOffset){
	_tcpPos[0] = xOffset;
	_tcpPos[1] = yOffset;
	_tcpPos[2] = zOffset;
	
	// Calculate new length and angle of last axis (since eef is connected to it statically)
    c_new = sqrt(pow(_tcpPos[1],2) + pow(c+_tcpPos[0],2));
    beta = asin(fabs(_tcpPos[1]/c_new));
	
	c_newSQ = pow(c_new,2);
	bSQ = pow(b,2);
}