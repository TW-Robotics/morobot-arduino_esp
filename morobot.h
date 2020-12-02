/**
 *  \class 	morobotClass
 *  \brief 	morobot base class for microcontrollers such as Arduino or ESP32
 *  @file 	morobot.h
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
			float getSpeed(uint8_t servoId);
			float getTemp(uint8_t servoId);
			float getVoltage(uint8_t servoId);
			float getCurrent(uint8_t servoId);
			
			void moveToAngle(uint8_t servoId, long angle);
			void moveToAngle(uint8_t servoId, long angle, uint8_t speedRPM, bool checkValidity=true);
			void moveToAngles(long angles[]);
			void moveToAngles(long angles[], uint8_t speedRPM);
			void moveAngle(uint8_t servoId, long angle);
			void moveAngle(uint8_t servoId, long angle, uint8_t speedRPM, bool checkValidity=true);
			void moveAngles(long angles[]);
			void moveAngles(long angles[], uint8_t speedRPM);
			bool moveToPosition(float x, float y, float z);
			bool moveXYZ(float xOffset, float yOffset, float zOffset);
			bool moveInDirection(char axis, float value);
			
			void printAngles(long angles[]);
		protected:
			virtual bool calculateAngles(float x, float y, float z);
			virtual void updateTCPpose();
			void autoCalibrateLinearAxis(uint8_t servoId, uint8_t maxMotorCurrent=25);
		private:
			bool isReady();
 */

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
		/**
		 *  \brief Constructor of morobot class
		 *  \param [in] numSmartServos Number of smart servos of the robot
		 */
		morobotClass(uint8_t numSmartServos);
		
		/**
		 *  \brief Starts the communication with the smartservos of the robot
		 *  \param [in] stream Name of serial port (e.g. "Serial1")
		 *  \details The names of the Serial ports for Arduino controllers can be found here: https://www.arduino.cc/reference/en/language/functions/communication/serial/
		 */
		void begin(const char* stream);
		
		/**
		 *  \brief Sets the current position as origin (zero position)
		 *  \details Call this function after bringing the motors into their initial (zero position) to store it permanently as 0 degrees
		 */
		void setZero();
		
		/**
		 *  \brief Moves all motors to their origin (zero position)
		 *  \details Only call this function after calibration (Setting a zero position for all motors using setZero())
		 */
		void moveHome();
		
		/**
		 *  \brief Sets the speed in RPM (rounds per minute) for all motors. Does not move a motor.
		 *  \param [in] speed Desired velocity of the motors in RPM (rounds per minute). Values accepted between 1 and 50.
		 *  \details Use this function to set a default speed for all motors so you do not have to give a speed every time the robot should move.
		 *  		 If a different speed is given in a movement-function, this speed will be used only for this movement.
		 */
		void setSpeedRPM(uint8_t speed);
		
		/**
		 *  \brief Sets the position of the TCP (tool center point) with respect to the center of the flange of the last robot axis.
		 *  \param [in] xOffset Offset in x-direction
		 *  \param [in] yOffset Offset in y-direction
		 *  \param [in] zOffset Offset in z-direction
		 *  \details Virtual function. Defined individually for each robot type in the respective child classes.
		 *  		 This information is necessary to calculate the inverse kinematics correctly.
		 */
		virtual void setTCPoffset(float xOffset, float yOffset, float zOffset)=0;
		
		/**
		 *  \brief Checks if a given angle can be reached by the joint. Each joint has a specific limit to protect the robot's mechanics.
		 *  
		 *  \param [in] servoId Number of motor to move (first motor has ID 0)
		 *  \param [in] angle Angle to move the robot to in degrees
		 *  \return Returns true if the position is reachable; false if it is not.
		 *  \details Virtual function. Defined individually for each robot type in the respective child classes.
		 */
		virtual bool checkIfAngleValid(uint8_t servoId, float angle)=0;
		
		/* BREAKS */
		/**
		 *  \brief Sets the breaks of all motors. Axes cannot be moved after calling this function.
		 *  \details After moving a motor with a movement-function, the motor is breaked at final position automatically. 
		 */
		void setBreaks();
		
		/**
		 *  \brief Releases the breaks of all motors. Axes can be moved freely after calling this function.
		 *  \details After moving a motor with a movement-function, the motor is breaked at final position automatically.
		 *  		 Use this function to release breaks to move the motor with your hands.
		 */
		void releaseBreaks();

		/* ROBOT STATUS */
		/**
		 *  \brief Sets internal variables which indicate, that the robot should not be moved at the moment.
		 *  \details Use setIdle() to indicate, that the robot can be moved.
		 *  		 Use waitUntilIsReady() to wait until the robot is idle (no motor moves) before continuing.
		 */
		void setBusy();
		
		/**
		 *  \brief Sets internal variables which indicate, that the robot can be moved.
		 *  \details Use setBusy() to indicate, that the robot should not be moved.
		 */
		void setIdle();
		
		/**
		 *  \brief Waits until the robot is ready for new commants (all motors have stopped moving) or a timeout occurs.
		 *  \details Function sets the robot idle only when all motors have stopped moving or a timeout occurs.
		 *  		 If a timeout occurs this is printed to the serial monitor.
		 */
		void waitUntilIsReady();
		
		/**
		 *  \brief Check if a given smart servo is moving at the moment.
		 *  \param [in] servoId Number of motor to check (first motor has ID 0)
		 *  \return Returns True if motor is moving; False if motor is not moving.
		 */
		bool checkIfMotorMoves(uint8_t servoId);
		
		/* GETTERS */
		/**
		 *  \brief Returns angle-position of motor in degrees.
		 *  \param [in] servoId Number of motor (first motor has ID 0)
		 *  \return Angle-positon in degrees.
		 */
		long getActAngle(uint8_t servoId);
		
		/**
		 *  \brief Returns position of TCP in mm in given axis (in robot base frame).
		 *  \param [in] axis Possible parameters: 'x', 'y', 'z'
		 *  \return Position of TCP in mm in given axis.
		 */
		float getActPosition(char axis);
		
		/**
		 *  \brief Returns current speed of motor in RPM (rounds per minute).
		 *  \param [in] servoId Number of motor (first motor has ID 0)
		 *  \return Current speed of motor in RPM.
		 */
		float getSpeed(uint8_t servoId);
		
		/**
		 *  \brief Returns current temperature of motor in degrees Celsius.
		 *  \param [in] servoId Number of motor (first motor has ID 0)
		 *  \return Current temperature of motor in degrees Celsius.
		 */
		float getTemp(uint8_t servoId);
		
		/**
		 *  \brief Returns current voltage of motor.
		 *  \param [in] servoId Number of motor (first motor has ID 0)
		 *  \return Current voltage of motor.
		 */
		float getVoltage(uint8_t servoId);
		
		/**
		 *  \brief Returns current current consumption of motor in Ampere.
		 *  \param [in] servoId Number of motor (first motor has ID 0)
		 *  \return Current current consumption of motor in Ampere.
		 */
		float getCurrent(uint8_t servoId);
		
		/* MOVEMENTS */
		/**
		 *  \brief Moves a motor to a desired angle (absolute movement).
		 *  \param [in] servoId Number of motor (first motor has ID 0)
		 *  \param [in] angle Desired goal angle in degrees.
		 *  \param [in] speedRPM (Optional) Desired velocity of the motor in RPM (rounds per minute). Values accepted between 1 and 50. If no speed is given, the preset default speed is used.
		 *  \param [in] checkValidity (Optional) Set this variable to "false" if you don't want to check if the angle value is valid (e.g. necessary for calibration)
		 *  \details Checks if the angle is valid before moving if checkValidity is not given or true.
		 */
		void moveToAngle(uint8_t servoId, long angle);
		void moveToAngle(uint8_t servoId, long angle, uint8_t speedRPM, bool checkValidity=true);
		
		/**
		 *  \brief Moves all motors to desired angles (Moves the whole robot) - absolute movement.
		 *  \param [in] angles[] Desired goal angles in degrees.
		 *  \param [in] speedRPM (Optional) Desired velocity of the motor in RPM (rounds per minute). Values accepted between 1 and 50. If no speed is given, the preset default speed is used.
		 *  \details Waits until the robot is ready to use (no motor moves) before moving.
		 *  		 Checks if the angles are valid before moving.
		 */
		void moveToAngles(long angles[]);
		void moveToAngles(long angles[], uint8_t speedRPM);

		/**
		 *  \brief Moves a motor by a desired angle (relative movement).
		 *  \param [in] servoId Number of motor (first motor has ID 0)
		 *  \param [in] angle Desired angle in degrees to move robot by.
		 *  \param [in] speedRPM (Optional) Desired velocity of the motor in RPM (rounds per minute). Values accepted between 1 and 50. If no speed is given, the preset default speed is used.
		 *  \param [in] checkValidity (Optional) Set this variable to "false" if you don't want to check if the angle value is valid (e.g. necessary for calibration)
		 *  \details Checks if the goal angle is valid before moving if checkValidity is not given or true.
		 */
		void moveAngle(uint8_t servoId, long angle);
		void moveAngle(uint8_t servoId, long angle, uint8_t speedRPM, bool checkValidity=true);
		
		/**
		 *  \brief Moves all motors by desired angles (Moves the whole robot) - relative movement.
		 *  \param [in] angles[] Desired angles in degrees to move robot by.
		 *  \param [in] speedRPM (Optional) Desired velocity of the motor in RPM (rounds per minute). Values accepted between 1 and 50. If no speed is given, the preset default speed is used.
		 *  \details Waits until the robot is ready to use (no motor moves) before moving.
		 *  		 Checks if the angles are valid before moving.
		 */
		void moveAngles(long angles[]);
		void moveAngles(long angles[], uint8_t speedRPM);
		
		/**
		 *  \brief Moves the TCP (tool center point) of the robot to a desired position.
		 *  \param [in] x Desired x-coordinate of the TCP in mm (in base frame)
		 *  \param [in] y Desired y-coordinate of the TCP in mm (in base frame)
		 *  \param [in] z Desired z-coordinate of the TCP in mm (in base frame)
		 *  \return Returns true if the position is reachable; false if it is not.
		 *  \details Calls child class to solve inverse kinematics and moves the robot to the position.
		 */
		bool moveToPosition(float x, float y, float z);
		
		/**
		 *  \brief Moves the TCP (tool center point) of the robot by given axis-values.
		 *  \param [in] xOffset Desired x-value to move the TCP by in mm
		 *  \param [in] yOffset Desired y-value to move the TCP by in mm
		 *  \param [in] zOffset Desired z-value to move the TCP by in mm
		 *  \return Returns true if the position is reachable; false if it is not.
		 */
		bool moveXYZ(float xOffset, float yOffset, float zOffset);
		
		/**
		 *  \brief Moves the TCP (tool center point) of the robot by given value in one axis.
		 *  \param [in] axis Axis to move the robot in. Possible parameters: 'x', 'y', 'z'
		 *  \param [in] value Value by which the robot should be moved in mm
		 *  \return Returns true if the position is reachable; false if it is not.
		 */
		bool moveInDirection(char axis, float value);
		
		/* HELPER */
		/**
		 *  \brief Prints an array of angles to the servial monitor.
		 *  \param [in] angles[] Angle values to print.
		 */
		void printAngles(long angles[]);
		
		/**
		 *  \brief Returns angle converted from rad into deg
		 *  \param [in] angle Angle to convert in radians
		 *  \return Returns angle in degree
		 */
		float convertToDegrees(float angle);
		
		/* PUBLIC VARIABLES */
		MakeblockSmartServo smartServos;	// Makeblock smartservo object
	protected:
		/**
		 *  \brief Uses given coordinates to calculate the motor angles to reach this position (Solve inverse kinematics).
		 *  \param [in] x Desired x-position of TCP
		 *  \param [in] y Desired x-position of TCP
		 *  \param [in] z Desired x-position of TCP
		 *  \return Returns true if the position is reachable; false if it is not.
		 *  \details Virtual function. Defined individually for each robot type in the respective child classes.
		 *  		 This function does only calculate the angles of the motors and stores them internally.
		 *  		 Use moveToPosition(x,y,z) to actually move the robot.
		 */
		virtual bool calculateAngles(float x, float y, float z)=0;

		/**
		 *  \brief Re-calculates the internally stored robot TCP position (Solves forward kinematics).
		 *  \details Virtual function. Defined individually for each robot type in the respective child classes.
		 *  		 This function does calculate and store the TCP position depending on the current motor angles.
		 */
		virtual void updateTCPpose()=0;
		
		/**
		 *  \brief Calibrates a linear axis by increasing the angle until a current limit is reached
		 *  \param [in] servoId Number of motor to calibrate (first motor has ID 0)
		 *  \param [in] maxMotorCurrent (Optional) Current limit at which zero position is reached an calibration stops
		 */
		void autoCalibrateLinearAxis(uint8_t servoId, uint8_t maxMotorCurrent=25);
	
		uint8_t _numSmartServos;			// Number of smart servos of robot
		uint8_t _speedRPM;					// Default speed (used if movement-funtions to not provide specific speed)
		float _actPos[3];					// Robot TCP position (in base frame)
		float _actOri[3];					// Robot TCP orientation (rotation in degrees around base frame)
		bool _angleReached[NUM_MAX_SERVOS];	// Variables that indicate if a motor is busy (is moving and has not reached final position)
		float _goalAngles[NUM_MAX_SERVOS];	// Variable for inverse kinematics to store goal Angles of the motors
		Stream* _port;						// Port used for communication with the robot (e.g. Serial1)
	private:
		/**
		 *  \brief Checks if the robot is busy or idle.
		 *  \return Returns true if the robot is idle; false if the robot is busy
		 */
		bool isReady();
};

#endif