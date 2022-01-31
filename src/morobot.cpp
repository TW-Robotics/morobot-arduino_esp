/**
 *  \class 	morobotClass
 *  \brief 	morobot base class for microcontrollers such as Arduino or ESP32
 *  @file 	morobot.cpp
 *  @author	Johannes Rauer FHTW
 *  @date	2020/11/27
 *  \par Method List:
 *  	public:
 *  		morobotClass(uint8_t numSmartServos);
			void begin(const char* stream);
			void setZero();
			void moveHome();
			void setSpeedRPM(uint8_t speed);
			virtual void setTCPoffset(float xOffset, float yOffset, float zOffset);
			virtual bool checkIfAngleValid(uint8_t servoId, float angle);
			
			void setBreaks();
			void releaseBreaks();

			void setBusy();
			void setIdle();
			void waitUntilIsReady();
			bool checkIfMotorMoves(uint8_t servoId);
			
			long getActAngle(uint8_t servoId);
			float getActPosition(char axis);
			float getActOrientation(char axis);
			float getSpeed(uint8_t servoId);
			float getTemp(uint8_t servoId);
			float getVoltage(uint8_t servoId);
			float getCurrent(uint8_t servoId);
			long getJointLimit(uint8_t servoId, bool limitNum);
			uint8_t getAxisLimit(char axis, bool limitNum);
			uint8_t getNumSmartServos();
			
			void moveToAngle(uint8_t servoId, long angle);
			void moveToAngle(uint8_t servoId, long angle, uint8_t speedRPM, bool checkValidity=true);
			void moveToAngles(long angles[]);
			void moveToAngles(long angles[], uint8_t speedRPM);
			void moveAngle(uint8_t servoId, long angle);
			void moveAngle(uint8_t servoId, long angle, uint8_t speedRPM, bool checkValidity=true);
			void moveAngles(long angles[]);
			void moveAngles(long angles[], uint8_t speedRPM);
			bool moveToPose(float x, float y, float z);
			bool moveXYZ(float xOffset, float yOffset, float zOffset);
			bool moveInDirection(char axis, float value);

			void trajectoryPlanning(float points[][3], int nrPoints, int continuousMovement = 0, int polynomOrder = 3);
			void moveLinear(float goalPoint[], int continuousMovement=1, float resolution=5,int useTrajectoryPlanning = 0);
			float calcPolynomThirdOrder(int startAngle, int endAngle, float startVel, float endVel, float time, float totalTime);
			float calcPolynomFifthOrder(int startAngle, int endAngle, float startVel, float endVel, float time, float totalTime);
			float calcIntermediateVelocity(float time, float q0, float q1, float q2);
			float calcPwm(float deg_per_sec);
			
			void printAngles(long angles[]);
			void printTCPpose();
			float convertToDeg(float angle);
			float convertToRad(float angle);
			virtual String getType();
		protected:
			virtual bool calculateAngles(float x, float y, float z);
			virtual void updateTCPpose(bool output);
			void autoCalibrateLinearAxis(uint8_t servoId, uint8_t maxMotorCurrent=25);
		private:
			bool isReady();
 */

#include "morobot.h"

morobotClass::morobotClass(uint8_t numSmartServos){
	if (numSmartServos > NUM_MAX_SERVOS){
		Serial.print(F("Too many motors! Maximum number of motors: "));
		Serial.println(NUM_MAX_SERVOS);
	}
	_numSmartServos = numSmartServos;
}

void morobotClass::begin(const char* stream){
	Serial.begin(115200);
	#if defined(ARDUINO_AVR_MEGA) || defined(ARDUINO_AVR_MEGA2560)
		if (stream == "Serial") {
			Serial.println(F("WARNING: Serial on Arduino Mega is connected to the USB-Controller, so you may get strange bytestings in the serial monitor!"));
			_port = &Serial;
		} else if (stream == "Serial1") {
			Serial1.begin(115200);
			_port = &Serial1;
		} else if (stream == "Serial2") {
			Serial2.begin(115200);
			_port = &Serial2;
		} else if (stream == "Serial3") {
			Serial3.begin(115200);
			_port = &Serial3;
		} else {
			Serial.println(F("ERROR: Serial-Parameter not valid. Choose 'Serial', 'Serial1', 'Serial2' or 'Serial3'."));
		}
		
	#elif defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MINI) || defined (ARDUINO_AVR_NANO)
		if (stream == "Serial") {
			Serial.println(F("WARNING: Serial on Arduino UNO is connected to the USB-Controller, so you may get strange bytestings in the serial monitor!"));
			_port = &Serial;
		} else {
			Serial.println(F("ERROR: Serial-Parameter not valid. Only 'Serial' possible."));
		}
		
	#elif defined(ARDUINO_AVR_LEONARDO) || defined(ARDUINO_AVR_MICRO) || defined(ARDUINO_AVR_YUN)
		if (stream == "Serial1") {
			Serial1.begin(115200);
			_port = &Serial1;
		} else {
			Serial.println(F("ERROR: Serial-Parameter not valid. Only 'Serial1' possible."));
		}
		
	#elif defined(ESP32)
		if (stream == "Serial") {
			Serial.println(F("WARNING: Serial on ESP32 is connected to the USB-Controller, so you may get strange bytestings in the serial monitor!"));
			_port = &Serial;
		} else if (stream == "Serial1") {
			Serial1.begin(115200, SERIAL_8N1, 18, 19);		// Map the serial pins to different pins since 9/10 are not mapped
			_port = &Serial1;
		} else if (stream == "Serial2") {
			Serial2.begin(115200);
			_port = &Serial2;
		} else {
			Serial.println(F("ERROR: Serial-Parameter not valid. Choose 'Serial1' or 'Serial2'."));
		}
		
	#elif defined(ESP8266)
		if (stream == "Serial") {
			Serial.println(F("WARNING: Serial on ESP32 is connected to the USB-Controller, so you may get strange bytestings in the serial monitor!"));
			_port = &Serial;
		} else {
			Serial.println(F("ERROR: Serial-Parameter not valid. Only 'Serial' possible."));
		}	
	#else
		#error "Board not supported"
	#endif
		
	smartServos.beginSerial(_port);
	delay(5);
	smartServos.assignDevIdRequest();
	delay(50);
	
	setTCPoffset(0, 0, 0);
	setSpeedRPM(25);
	updateTCPpose();

	Serial.println(F("Morobot initialized. Connection to motors established"));
}

void morobotClass::setZero(){
	for (uint8_t i=0; i<_numSmartServos; i++) smartServos.setZero(i+1);
	_tcpPoseIsValid = false;
}

void morobotClass::moveHome(){
	for (uint8_t i=0; i<_numSmartServos; i++) smartServos.setInitAngle(i+1, 0, 15);
	waitUntilIsReady();
	_tcpPoseIsValid = false;
}

void morobotClass::setSpeedRPM(uint8_t speed){
	_speedRPM = speed;
	
	// If the speed is bigger than the maximum speed, set it to maximum.
	// If the speed is smaller than then 1, set it to minimum.
	if (speed > SERVO_MAX_SPEED_RPM) _speedRPM = SERVO_MAX_SPEED_RPM;
	if (speed < 0) _speedRPM = 1;
}


/* BREAKS */
void morobotClass::setBreaks(){
	for (uint8_t i=0; i<_numSmartServos; i++) smartServos.setBreak(i+1, BREAK_BRAKED);
}

void morobotClass::releaseBreaks(){
	for (uint8_t i=0; i<_numSmartServos; i++) smartServos.setBreak(i+1, BREAK_LOOSE);
	_tcpPoseIsValid = false;
}


/* ROBOT STATUS */
void morobotClass::setBusy(){
	for (uint8_t i=0; i<_numSmartServos; i++) _angleReached[i] = false;
}

void morobotClass::setIdle(){
	for (uint8_t i=0; i<_numSmartServos; i++) _angleReached[i] = true;
}

void morobotClass::waitUntilIsReady(){
	if (!waitAfterEachMove) {
		setIdle();
		return;
	}
	setBusy();
	unsigned long startTime = millis();
	while (true){
		// Check if the robot is ready yet
		if (isReady() == true) break;
		// Stop waiting if the robot is not ready after a timeout occurs
		if ((millis() - startTime) > TIMEOUT_DELAY) {
			Serial.println(F("TIMEOUT OCCURED WHILE WAITING FOR ROBOT TO FINISH MOVEMENT!"));
			break;
		}
	}
}

bool morobotClass::checkIfMotorMoves(uint8_t servoId){
	long startPos = getActAngle(servoId);
	delay(150);
	if (startPos != getActAngle(servoId)) return true;
	return false;
}


/* GETTERS */
long morobotClass::getActAngle(uint8_t servoId){
	return smartServos.getAngleRequest(servoId+1);
}

float morobotClass::getActPosition(char axis){
	updateTCPpose();
	
	if (axis == 'x') return _actPos[0];
	else if (axis == 'y') return _actPos[1];
	else if (axis == 'z') return _actPos[2];
	else Serial.println(F("ERROR! Invalid axis in getActPosition();"));
}

float morobotClass::getActOrientation(char axis){
	updateTCPpose();
	
	if (axis == 'x') return _actOri[0];
	else if (axis == 'y') return _actOri[1];
	else if (axis == 'z') return _actOri[2];
	else Serial.println(F("ERROR! Invalid axis in getActOrientation();"));
}

float morobotClass::getSpeed(uint8_t servoId){
	return smartServos.getSpeedRequest(servoId+1);
}

float morobotClass::getTemp(uint8_t servoId){
	return smartServos.getTempRequest(servoId+1);
}

float morobotClass::getVoltage(uint8_t servoId){
	return smartServos.getVoltageRequest(servoId+1);
}

float morobotClass::getCurrent(uint8_t servoId){
	return smartServos.getCurrentRequest(servoId+1);
}

long morobotClass::getJointLimit(uint8_t servoId, bool limitNum){
	return _robotJointLimits[servoId][limitNum];
}

uint8_t morobotClass::getAxisLimit(char axis, bool limitNum){
	if (axis == 'x') return _robotAxisLimits[0][limitNum];
	else if (axis == 'y') return _robotAxisLimits[1][limitNum];
	else if (axis == 'z') return _robotAxisLimits[2][limitNum];
}

uint8_t morobotClass::getNumSmartServos(){
	return _numSmartServos;
}

/* MOVEMENTS */
void morobotClass::moveToAngle(uint8_t servoId, long angle){
	if (checkIfAngleValid(servoId, angle) == true) {
		smartServos.moveTo(servoId+1, angle, _speedRPM);
		_tcpPoseIsValid = false;
	}
}

void morobotClass::moveToAngle(uint8_t servoId, long angle, uint8_t speedRPM, bool checkValidity){
	if (checkValidity == false) {
		smartServos.moveTo(servoId+1, angle, speedRPM);
		_tcpPoseIsValid = false;
	} else if (checkIfAngleValid(servoId, angle) == true) {
		smartServos.moveTo(servoId+1, angle, speedRPM);
		_tcpPoseIsValid = false;
	}
}

void morobotClass::moveToAngles(long angles[]){
	waitUntilIsReady();
	Serial.print(F("Moving to [deg]: "));
	printAngles(angles);
	
	for (uint8_t i=0; i<_numSmartServos; i++) moveToAngle(i, angles[i]);
}

void morobotClass::moveToAngles(long angles[], uint8_t speedRPM){
	waitUntilIsReady();
	Serial.print(F("Moving to [deg]: "));
	printAngles(angles);
	
	for (uint8_t i=0; i<_numSmartServos; i++) moveToAngle(i, angles[i], speedRPM);
}

void morobotClass::moveToAngles(long phi0, long phi1, long phi2){
	long angles[3] = {phi0, phi1, phi2};
	moveToAngles(angles);
}

void morobotClass::moveAngle(uint8_t servoId, long angle){
	if (checkIfAngleValid(servoId, getActAngle(servoId)+angle) == true) {
		smartServos.move(servoId+1, angle, _speedRPM);
		_tcpPoseIsValid = false;
	}
}

void morobotClass::moveAngle(uint8_t servoId, long angle, uint8_t speedRPM, bool checkValidity){
	if (checkValidity == false) {
		smartServos.move(servoId+1, angle, speedRPM);
		_tcpPoseIsValid = false;
	} else if (checkIfAngleValid(servoId, getActAngle(servoId)+angle) == true) {
		smartServos.move(servoId+1, angle, speedRPM);
		_tcpPoseIsValid = false;
	}
}

void morobotClass::moveAngles(long angles[]){
	waitUntilIsReady();
	Serial.print(F("Moving [deg]: "));
	printAngles(angles);

	for (uint8_t i=0; i<_numSmartServos; i++) moveAngle(i, angles[i]);
}

void morobotClass::moveAngles(long angles[], uint8_t speedRPM){
	waitUntilIsReady();
	Serial.print(F("Moving [deg]: "));
	printAngles(angles);

	for (uint8_t i=0; i<_numSmartServos; i++) moveAngle(i, angles[i], speedRPM);
}

bool morobotClass::moveToPose(float x, float y, float z){
	waitUntilIsReady();
	Serial.print(F("Moving to [mm]: "));
	Serial.print(x);
	Serial.print(", ");
	Serial.print(y);
	Serial.print(", ");
	Serial.println(z);
	
	updateTCPpose();
	if (calculateAngles(x, y, z) == false) return false;
	
	for (uint8_t i=0; i<_numSmartServos; i++) moveToAngle(i, _goalAngles[i]);
	
	// Update TCP-Pose
	_actPos[0] = x;
	_actPos[1] = y;
	_actPos[2] = z;
	_tcpPoseIsValid = true;
	
	return true;
}

bool morobotClass::moveXYZ(float xOffset, float yOffset, float zOffset){
	updateTCPpose();
	return moveToPose(_actPos[0]+xOffset, _actPos[1]+yOffset, _actPos[2]+zOffset);
}

bool morobotClass::moveInDirection(char axis, float value){
	updateTCPpose();
	float goalxyz[3];
	goalxyz[0] = _actPos[0];
	goalxyz[1] = _actPos[1];
	goalxyz[2] = _actPos[2];
	
	if (axis == 'x') goalxyz[0] = goalxyz[0]+value;
	else if (axis == 'y') goalxyz[1] = goalxyz[1]+value;
	else if (axis == 'z') goalxyz[2] = goalxyz[2]+value;
	return moveToPose(goalxyz[0], goalxyz[1], goalxyz[2]);
}

//###############################
//----- trajectory planning -----
//###############################

float morobotClass::calcPwm(float deg_per_sec){
	float k=4.6109;
	float d=9.1866;
	float pwm=0;

	//calculate the PWM value from the velocity
	if(deg_per_sec<0){
		pwm=((deg_per_sec-d)/k);
	}else{
		d=12;
		pwm=((deg_per_sec+d)/k);
	}	

	return pwm;
}

float morobotClass::calcPolynomThirdOrder(int startAngle, int endAngle, float startVel, float endVel, float time, float totalTime){
	float diff = endAngle-startAngle;
	
	//set the start velocity
	if(startVel==0){
		if(diff>0){
			startVel=24;
		}else{
			startVel=-24;
		}
	}

	//calculate the velocity depending on time
	float c1 = startVel;
	float c2 = (3*diff-(2*startVel+endVel)*totalTime)/pow(totalTime,2);
	float c3 = (-2*diff+(startVel+endVel)*totalTime)/pow(totalTime,3);
	float theta_v = c1 + c2*time*2 + c3*pow(time,2)*3;

	//set a maximum velocity 
	float limit = 140;
	if(theta_v >limit){
		theta_v=limit;
	}else if(theta_v<-limit){
		theta_v=-limit;
	}
	
	//calculate pwm value
	return calcPwm(theta_v);
}

float morobotClass::calcPolynomFifthOrder(int startAngle, int endAngle, float startVel, float endVel, float time, float totalTime){
	float diff = endAngle-startAngle;
	
	//set the start velocity
	if(startVel==0){
		if(diff>0){
			startVel=24;
		}else{
			startVel=-24;
		}
	}
	
	//calculate the velocity depending on time
	float c1 = startVel;
	float c2 = 0;
	float c3 = (20*diff-(8*endVel+12*startVel)*totalTime)/(2*pow(totalTime,3));
	float c4 = (-30*diff+(14*endVel+16*startVel)*totalTime)/(2*pow(totalTime,4));
	float c5 = (12*diff-6*(endVel+startVel)*totalTime)/(2*pow(totalTime,5));
	float theta_v = c1 + c2*time*2 + c3*pow(time,2)*3 + c4*pow(time,3)*4 + c5*pow(time,4)*5;

	//set a maximum velocity
	float limit = 140;
	if(theta_v >limit){
		theta_v=limit;
	}else if(theta_v<-limit){
		theta_v=-limit;
	}

	//calculate pwm value
	return calcPwm(theta_v);
}

float morobotClass::calcIntermediateVelocity(float time, float q0, float q1, float q2){
	float d1 = (q1-q0)/time;
	float d2 = (q2-q1)/time;
	float v = 0;

	if(signbit(d1) == signbit(d2)){
		v = (0.5*(d1+d2));
	}else{
		v=0; //or 24
	}
	return v;
}

void morobotClass::trajectoryPlanning(float points[][3], int nrPoints, int continuousMovement = 0, int polynomOrder = 3){
	int jointValues[nrPoints][_numSmartServos]={};	//array for joint angles for all points
	float jointVelocities[_numSmartServos]={};		//array for joint velocities
	int startJointAngles[_numSmartServos]={};		//array for start angles	
	int nrIterations = 10; 
	float minPwmVel=25.58854; //[°/sec]
	int k=0;
	static uint32_t starttime=millis();

	//step 1: calculate inverse kinematics to get joint angles for all points
	for(int i=0; i<nrPoints; ++i){
		calculateAngles(points[i][0], points[i][1], points[i][2]);	//calculate inverse kinematics for the x,y and z pose
		Serial.print(i);
		Serial.print(":");
		for(int j=0;j<_numSmartServos; ++j){	//loop over the motors 
			jointValues[i][j]=_goalAngles[j];	//save the calculated motor angle in jointValues
			Serial.print(" ");
			Serial.print(jointValues[i][j]);
		}
		Serial.print("\n");
	}

	float max_total_time=0;
	int min_dis=360;
	int joint = 0;
	float end_vel[_numSmartServos]={};		//continuous movment - end velocity 
	float start_vel[_numSmartServos]={};	//continuous movment - start velocity 

	for(int i=0; i< nrPoints; ++i){	//loop over all points
		Serial.print("\n\npoint: ");
		Serial.println(i+1);
		min_dis=360;
		for(int j=0; j<_numSmartServos; ++j){
			start_vel[j]=24;
			end_vel[j] = 24;
		}

		//step 2: synchronisation - calculate max time for each point
		//check for the minimum distance that has to be travelled -> must be travelled with at least the min velocity
		for(int j=0; j<_numSmartServos;++j){ 	//loop over all joints
			startJointAngles[j]=getActAngle(j); //get current angle
			Serial.print("j: ");
			Serial.print(j);
			Serial.print(" start: ");
			Serial.print(startJointAngles[j]);
			Serial.print(" end: ");
			Serial.print(jointValues[i][j]);
			Serial.print(" diff: ");
			Serial.println(jointValues[i][j]-startJointAngles[j]);
			if(abs(startJointAngles[j]-jointValues[i][j])<min_dis && abs(startJointAngles[j]-jointValues[i][j])!=0){	//find the min distance that has to be travelled by a joint
				min_dis=abs(startJointAngles[j]-jointValues[i][j]);
				joint = j;
			}
		}
		max_total_time=min_dis/minPwmVel;	//divide the min distance by the min velocity to get the maximum total time (if the total time is higher, the calculated velocity is smaller than the min velocity and the corresponding motor won't move)
		Serial.print("Min dis: ");
		Serial.print(min_dis);
		Serial.print(" time: ");
		Serial.print(max_total_time);
		Serial.print(" joint: ");
		Serial.println(joint);
		
		//step 2a: continuous movment - calculate start and end-velocity
		if(continuousMovement==1){
			for(int j=0; j<_numSmartServos; ++j){
				start_vel[j]=end_vel[j];
				if(i!=nrPoints-1){
					end_vel[j] = calcIntermediateVelocity(max_total_time, startJointAngles[j], jointValues[i][j], jointValues[i+1][j]);
				}else{
					end_vel[j] = 0;
				}
				Serial.print("j: ");
				Serial.print(j);
				Serial.print(" start v: ");
				Serial.print(start_vel[j]);
				Serial.print(" end v: ");
				Serial.println(end_vel[j]);
			}
		}

		//step 3: calculate velocities with trajectory planning methods
		starttime=millis();
		while(k<nrIterations){	//loop over timeslots per point	
			if((millis()-starttime)>((max_total_time/nrIterations)*1000)){	//calculate the joint velocities at the start of every time slot (max_total_time/nrIterations)			
				starttime=millis();

				for(int j=0; j<_numSmartServos;++j){ //loop over servos
					//calculate velocity for current time slot
					if(polynomOrder==5){
						if(continuousMovement==1){
							jointVelocities[j]=calcPolynomFifthOrder(startJointAngles[j], jointValues[i][j], start_vel[j], end_vel[j], max_total_time/nrIterations*(k+1), max_total_time);
						}else{
							jointVelocities[j]=calcPolynomFifthOrder(startJointAngles[j], jointValues[i][j], 24, 10, max_total_time/nrIterations*(k+1), max_total_time); 
						}
					}else{
						if(continuousMovement==1){
							jointVelocities[j]=calcPolynomThirdOrder(startJointAngles[j], jointValues[i][j], start_vel[j], end_vel[j], max_total_time/nrIterations*(k+1), max_total_time);
						}else{
							jointVelocities[j]=calcPolynomThirdOrder(startJointAngles[j], jointValues[i][j], 24, 0, max_total_time/nrIterations*(k+1), max_total_time); 
						}
					}
					
					smartServos.setPwmMove(j+1, jointVelocities[j]);	//send velocity to servo
				}
				++k;
			}
			
		}
		k=0;

		if(continuousMovement!=1){
			delay(1000);
			printTCPpose();
		}

	}

	printTCPpose();
}

void morobotClass::moveLinear(float goalPoint[], int continuousMovement=1, float resolution=5,int useTrajectoryPlanning = 0){
	//Step 1: calculate straight line in 3D-Space:
	//X(s)=A+sv, where A=Startpoint, V=Goalpoint-Startpoint (direction), s=Intervall*(1/numberOfIntervalls)
	float A[3]={}; 
	float v[3]={}; 

	updateTCPpose();
	for(int i=0; i<3; ++i){
		A[i]=_actPos[i];
		v[i]=goalPoint[i]-_actPos[i];
		Serial.print(i);
		Serial.print(" akt: ");
		Serial.print(_actPos[i]);
		Serial.print(" distance ");
		Serial.print(": ");
		Serial.println(v[i]);
	}

	//Step 2: calculate length of line
	float len = sqrt(pow(v[0],2)+pow(v[1],2)+pow(v[2],2));
	Serial.print("Length of line: ");
	Serial.println(len);

	//Step 3: determine number of intervalls and calculate s
	int nrIntervalls = len/resolution;
	float s = 1/float(nrIntervalls);

	Serial.print("Number of intervalls: ");
	Serial.println(nrIntervalls);

	//Step 4: create Points
	float points[nrIntervalls][3] = {};
	Serial.println("---\nPoints - (i: x, y, z)");
	for(int i=0; i<nrIntervalls; ++i){
		Serial.print(i);
		Serial.print(":");
		for(int j=0; j<3; ++j){
			points[i][j] = A[j]+(i+1)*s*v[j];
			Serial.print(" ");
			Serial.print(points[i][j]);
		}
		Serial.print("\n");
	}

	//--------------------------------------
	//use trajectory planning for the movment
	if(useTrajectoryPlanning == 1){
		trajectoryPlanning(points, sizeof(points)/sizeof(points[0]), 1, 3);
		return;
	}

	//--------------------------------------
	//Step 5: calculate inverse kinematics for every point
	int jointValues[nrIntervalls+1][_numSmartServos]={};
	Serial.println("\nAngles - (i: 1 2 3 [°])");
	for(int i=0; i<nrIntervalls; ++i){
		calculateAngles(points[i][0], points[i][1], points[i][2]);	//calculate inverse kinematics for the x,y and rot_z pose
		Serial.print(i);
		Serial.print(":");
		for(int j=0;j<_numSmartServos; ++j){	//loop over the motors 
			jointValues[i][j]=_goalAngles[j];	//save the calculated motor angle in jointValues
			//print angles
			//Serial.print(j);
			Serial.print(" ");
			Serial.print(jointValues[i][j]);
		}
		Serial.print("\n");
	}

	//Step 6: move between points
	float minPwmVel=25.58854; //[°/sec]
	float max_total_time=0;
	int max_dis=0;
	int unfinished = 0;	//number of motors that have not reached their target position
	int joint_max_dis=0;
	int dis[_numSmartServos]={0};
	float jointDistances[_numSmartServos]={0};
	int startJointAngles[_numSmartServos]={};
	static uint32_t starttime=millis();
	bool while_loop=true;

	//loop over points
	for(int i=0; i<nrIntervalls; ++i){
		//calculate max distance
		max_dis=0;
		Serial.print("-------\nintervall: ");
		Serial.println(i);

		for(int j=0; j<_numSmartServos;++j){ 	//loop over all joints
			startJointAngles[j]=getActAngle(j); //get current angle
			Serial.print("j: ");
			Serial.print(j);
			Serial.print(" start: ");
			Serial.print(startJointAngles[j]);
			Serial.print(" end: ");
			Serial.print(jointValues[i][j]);
			Serial.print(" diff: ");
			Serial.println(startJointAngles[j]-jointValues[i][j]);
			jointDistances[j]=jointValues[i][j]-startJointAngles[j];

			if(abs(jointDistances[j])>max_dis){	//find the max distance that has to be travelled by a joint
				max_dis=abs(jointDistances[j]);
				joint_max_dis=j;
			}
		}
		
		//"joint_max_dis" takes the longest
		Serial.print("joint with max distance: ");
		Serial.println(joint_max_dis);

		//send velocities to motors
		for(int j=0;j<_numSmartServos;++j){
			if(jointDistances[j]>0){
				smartServos.setPwmMove(j+1, 8);
			}else{
				smartServos.setPwmMove(j+1, -8);
			}
			
		}

		//helper: discontinuous movement - set values in the array to 1 - once a motor has reached its intermediate goal position, the value is set to zero
		//once all values are zero, the next point is approached
		for(int j=0; j<_numSmartServos;++j){
			dis[j]=1;
		}

		starttime=millis();
		while(while_loop==true){
			for(int j=0; j<_numSmartServos;++j){ 	//loop over all joints
				if(continuousMovement==1 && (abs(getActAngle(joint_max_dis))>abs(jointValues[i][joint_max_dis]-1))){
					while_loop=false;
					break;
				}

				if(dis[j]!=0 && jointDistances[j]>0){
					if(abs(getActAngle(j))>=abs(jointValues[i][j])){
						smartServos.setPwmMove(j+1, 0);
						dis[j]=0;
					}

				}else if(dis[j]!=0){
					if(abs(getActAngle(j))<=abs(jointValues[i][j])){
						smartServos.setPwmMove(j+1, 0);
						dis[j]=0;
					}
				}
			}

			unfinished = 0;
			for(int j=0; j<3;++j){
				unfinished += dis[j];
			}

			if(unfinished==0){
				if(continuousMovement==0){
					printTCPpose();
				}
				break;
			}
			
		}
		while_loop=true;
		Serial.print("\n");
	}

	printTCPpose();
}

/* HELPER */
void morobotClass::printAngles(long angles[]){
	for (uint8_t i=0; i<_numSmartServos; i++) {
		Serial.print(angles[i]);
		if (i != _numSmartServos-1) Serial.print(", ");
		else Serial.println(" ");
	}
}

void morobotClass::printTCPpose(){
	updateTCPpose();
	Serial.print(F("TCP-Pose x, y, z [mm]: "));
	Serial.print(_actPos[0]);
	Serial.print(", ");
	Serial.print(_actPos[1]);
	Serial.print(", ");
	Serial.print(_actPos[2]);
	Serial.print(F("; Orientation around z-axis [degrees]: "));
	Serial.println(_actOri[2]);
}

float morobotClass::convertToDeg(float angle){
	return angle*180/M_PI;
}

float morobotClass::convertToRad(float angle){
	return angle*M_PI/180;
}

/* PROTECTED */
void morobotClass::autoCalibrateLinearAxis(uint8_t servoId, uint8_t maxMotorCurrent){
	while(true){
		moveAngle(servoId, -2, 1, false);
		if (getCurrent(servoId) > 25) break;
	}
	smartServos.setZero(servoId+1);
	Serial.println(F("Linear axis set zero!"));
}

bool morobotClass::checkForNANerror(uint8_t servoId, float angle){
	// The values are NAN if the inverse kinematics does not provide a solution
	if(isnan(angle)){
		Serial.print(F("Angle for motor "));
		Serial.print(servoId);
		Serial.println(F(" is NAN!"));
		_tcpPoseIsValid = false;
		return false;
	}
	return true;
}

void morobotClass::printInvalidAngleError(uint8_t servoId, float angle){
	// Moving the motors out of the joint limits may harm the robot's mechanics
	Serial.print(F("Angle for motor "));
	Serial.print(servoId);
	Serial.print(F(" is invalid! ("));
	Serial.print(angle);
	Serial.println(F(" degrees)."));
	_tcpPoseIsValid = false;
}

/* ROBOT STATUS PRIVATE */
bool morobotClass::isReady(){
	for (uint8_t i=0; i<_numSmartServos; i++) {
		if (_angleReached[i] == false) {
			if (checkIfMotorMoves(i) == false) continue;
			return false;
		}
	}
	return true;
}
