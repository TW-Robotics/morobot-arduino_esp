/**
 *  \file: trajectory_planning.ino
 *  \brief: examples for the implementation of trajectory planning
 *  @author  Claudia Holzgethan FHTW
 *  @date 2021/10/07
 *  
 *  Hardware:     - Arduino Mega (or similar microcontroller)
 *          - morobot RRR
 *          - Powersupply 9-12V 5A (or more)
 *  Connections:  - Powersupply to Arduino hollow connector
 *          - First smart servo of robot to Arduino:
 *            - Red cable to Vin
 *            - Black cable to GND
 *            - Yellow cable to pin 16 (TX2)
 *            - White calbe to pin 17 (RX2)
 */

#define MOROBOT_TYPE   morobot_s_rrr // morobot_s_rrr, morobot_s_rrp, morobot_2d, morobot_3d, morobot_p
#define SERIAL_PORT   "Serial2"   // "Serial", "Serial1", "Serial2", "Serial3" (not all supported for all microcontroller - see readme)

#include <morobot.h>

MOROBOT_TYPE morobot;    // And change the class-name here

void setup() {
  morobot.begin(SERIAL_PORT);
  morobot.setSpeedRPM(10);
  morobot.moveHome();        // Move the robot into initial position
  morobot.setZero();        // Set the axes zero when move in
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(500);
  
  static int i=1;     // choose an example
  static int show_all = 0;
  /* 1 ... point-to-point movement with third-order polynomials
   * 2 ... point-to-point movement with fifth-order polynomials
   * 3 ... continuous point-to-point/ multipoint movement with third-order polynomials
   * 4 ... original jerky movement
   * 5 ... continuous linear movement
   * 6 ... linear movement with pauses at each intermediate point
   * 7 ... linear movement with trajectory planning (not recommended with current motors)
   */
  
  if(i==1){
    //point-to-point movement with third-order polynomials
    Serial.println("\n---\npoint-to-point movement with third-order polynomials\n---\n");
    //                     x        y     z
    float poses[][3] = {{161.37, -113.63, 0},
                        {188.92,   -3.77, 0},
                        {137.38,   67.23, 0},
                        { 91.10,  110.04, 0}, 
                        {209,       0.0,  0}};

    morobot.trajectoryPlanning(poses, sizeof(poses)/sizeof(poses[0]), 0, 3);

    if(show_all==1){
      i+=1;
      delay(1000);
      morobot.moveHome();        // Move the robot into initial position
      delay(2000);
    }else{
      i=0;
    }
    
  }else if(i==2){
    //point-to-point movement with fifth-order polynomials
    Serial.println("\n---\npoint-to-point movement with fifth-order polynomials\n---\n");
    //                     x        y     z
    float poses[][3] = {{161.37, -113.63, 0},
                        {188.92,   -3.77, 0}};
                        
    morobot.trajectoryPlanning(poses, 2, 0, 5);
    
    if(show_all==1){
      i+=1;
      delay(1000);
      morobot.moveHome();        // Move the robot into initial position
      delay(2000);
    }else{
      i=0;
    }
    
  }else if(i==3){
    //continuous point-to-point/ multipoint movement with third-order polynomials
    Serial.println("\n---\ncontinuous point-to-point/ multipoint movement with third-order polynomials\n---\n");
    //                     x        y     z
    float poses[][3] = {{161.37, -113.63, 0},
                        {188.92,   -3.77, 0},
                        {137.38,   67.23, 0},
                        { 91.10,  110.04, 0}, 
                        {209,       0.0,  0}};

    morobot.trajectoryPlanning(poses, 5, 1, 3);
    
    if(show_all==1){
      i+=1;
      delay(1000);
      morobot.moveHome();        // Move the robot into initial position
      delay(2000);
    }else{
      i=0;
    }
    
  }else if(i==4){
    //original jerky movement
    Serial.println("\n---\noriginal jerky movement\n---\n");
    //                     x        y     z
    float poses[][3] = {{161.37, -113.63, 0},
                        {188.92,   -3.77, 0}};
    Serial.print(sizeof(poses)/sizeof(poses[0]));
    for(int j=0; j<sizeof(poses)/sizeof(poses[0]); j++){
      morobot.moveToPose(poses[j][0], poses[j][1], poses[j][2]);
    }
    
    if(show_all==1){
      i+=1;
      delay(1000);
      morobot.moveHome();        // Move the robot into initial position
      delay(2000);
    }else{
      i=0;
    }
    
  }else if(i==5){
    //continuous linear movement
    Serial.println("\n---\ncontinuous linear movement\n---\n");
    //                    x        y     z
    float end_pose[] = {161.37, -113.63, 0};
    morobot.moveLinear(end_pose, 1, 10);
    
    if(show_all==1){
      i+=1;
      delay(1000);
      morobot.moveHome();        // Move the robot into initial position
      delay(2000);
    }else{
      i=0;
    }
    
  }else if(i==6){
    //linear movement with pauses at each intermediate point
    Serial.println("\n---\nlinear movement with pauses at each intermediate point\n---\n");
    //                    x        y     z
    float end_pose[] = {161.37, -113.63, 0};
    morobot.moveLinear(end_pose, 0);
    
    if(show_all==1){
      i+=1;
      delay(1000);
      morobot.moveHome();        // Move the robot into initial position
      delay(2000);
    }else{
      i=0;
    }
    
  }else if(i==7){
    //linear movement with trajectory planning 
    Serial.println("\n---\nlinear movement with trajectory planning\n---\n");
    //                    x        y     z
    float end_pose[] = {161.37, -113.63, 0};
    morobot.moveLinear(end_pose, 0, 5, 1);
    
    if(show_all==1){
      i+=1;
      delay(1000);
      morobot.moveHome();        // Move the robot into initial position
      delay(2000);
    }else{
      i=0;
    }
  }  
  
}
