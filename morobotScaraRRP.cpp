#include <morobotScaraRRP.h>

void morobotScaraRRP::setTCPoffset(float xOffset, float yOffset, float zOffset){
	_tcpOffset[0] = xOffset;
	_tcpOffset[1] = yOffset;
	_tcpOffset[2] = zOffset;
	
	// Calculate new length and angle of last axis (since eef is connected to it statically)
    c_new = sqrt(pow(_tcpOffset[1],2) + pow(c+_tcpOffset[0],2));
    beta = asin(fabs(_tcpOffset[1]/c_new));
	
	c_newSQ = pow(c_new,2);
	bSQ = pow(b,2);
}

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
	Serial.println(_actPos[0]);
	Serial.println(_actPos[1]);
	Serial.println(_actPos[2]);
}

bool morobotScaraRRP::checkIfAngleValid(uint8_t servoId, float angle){
	if(isnan(angle)){
		Serial.print("Angle for motor ");
		Serial.print(servoId);
		Serial.println(" is NAN!");
		return false;
	}
	
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

bool morobotScaraRRP::checkIfAnglesValid(float phi1, float phi2, float phi3){
	float angles[3] = {phi1, phi2, phi3};
	
	for (uint8_t i = 0; i < _numSmartServos; i++) if(checkIfAngleValid(i, angles[i]) == false) return false;
	return true;
}