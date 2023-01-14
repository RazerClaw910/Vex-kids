#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <iostream> 
#include "vex.h"
#include "robot-config.h"
//#include <"pid.h">
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Vision14             vision        5               
// piston               digital_out   D               
// PotentiometerV2F     potV2         F               
// LineTrackerH         line          H               
// Inertial             inertial      20              
// Vision17             vision        17              
// ---- END VEXCODE CONFIGURED DEVICES ----
 
using namespace vex;

competition Competition;
 
// Brain should be defined by default
//brain Brain;
 

// START V5 MACROS
#define waitUntil(condition)                                                   \
 do {                                                                         \
   wait(5, msec);                                                             \
 } while (!(condition))

#define repeat(iterations)                                                     \\
 for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS
 
 
// Robot configuration code.

 
 
 
/*vex-vision-config:begin*/

 
 
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
#pragma endregion VEXcode Generated Robot Configuration
 
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       {author}                                                  */
/*    Created:      {date}                                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
 
// Include the V5 Library
#include "vex.h"
 // Allows for easier use of the VEX Library
using namespace vex;

// Declares motors
controller Controller1 = controller(primary);
motor BR = motor(PORT1, ratio6_1, false);
 
motor FR = motor(PORT6, ratio6_1, false);
 
motor BL = motor(PORT11, ratio6_1, true);
 
motor FL = motor(PORT16, ratio6_1, true);
 
motor TR = motor(PORT10, ratio6_1, false);
 
motor TL = motor(PORT12, ratio6_1, true);
 
motor intake = motor(PORT21, ratio18_1, true);
 
motor punch = motor(PORT9, ratio18_1, false);

motor_group lDrive = motor_group(FL, BL, TL);
motor_group rDrive = motor_group(FR, BR, TR);

//intake.setVelocity(100,percent);
float drive;
float turn;
//wait(500,msec);
int err;
int lasterr=0;
int pidout;
int speed=0;

double limiter(double val, double min, double max) {
  // Return the minimum value if val is less than min
  if (val < min) {
    return min;
  // Return the maximum value if val is greater than max
  } else if (val > max) {
    return max;
  } else {
  // Otherwise, return val
    return val;
  }
}

// Return the average of the drive train posistion
double driveTrainPos() {
  return (rDrive.position(degrees) + lDrive.position(degrees))/2;
}

void driveStraight(double target, double accuracy) {
  // Resets motor encoders
  rDrive.setPosition(0,degrees);
  lDrive.setPosition(0,degrees);

  // k values
  const double kP = 0.004;
  const double kI = 0;
  const double kD = 0;

  const double rkP = 0.005;
  const double rkD = 0;

  double currentPos = driveTrainPos();
  double rTarget = 0;

  // driving pid variables
  double error = target - currentPos;
  double prevError = 0;
  double integral = 0;
  double derivative = 0;
  double output = 0;

  double rError=rTarget-Inertial.heading();
  double rLastError = 0;
  double rSpeed = Inertial.gyroRate(xaxis, dps);;
  double rOutput;

  if (rError>180) {
    rError-=360;
    }
  if (rError<-180) {
    rError+=360;
    }

  // Driving pid
  while (fabs(error) - 300 > accuracy) {
    currentPos = driveTrainPos();
    error = target - currentPos;

    rError=rTarget-Inertial.heading();

    if (rError>180) {
      rError-=360;
    }
    if (rError<-180) {
      rError+=360;
    }

    rSpeed = Inertial.gyroRate(xaxis, dps);
    rLastError = rError;
    //Brain.Screen.print(error);
    //Brain.Screen.newLine();
    output = (kP * error + kI * integral + kD * derivative);
    rOutput = 0;//(rkP * rError + rkD * rSpeed);
    //output = limiter(output, -12, 12); // limits the volt for the output


  
    rDrive.spin(forward, output - rOutput, volt);
    lDrive.spin(forward, output + rOutput, volt);

    

    integral += error; // not used currently
    derivative = error - prevError;
    prevError = error;
    wait(10, msec);
  }
}

void turningPid(double turnTarget, double turnAccuracy) {
  // Resets motor encoders
  rDrive.setPosition(0,degrees);
  lDrive.setPosition(0,degrees);

  // k values
  const double tkP = 0.06;
  const double tkI = 0;
  const double tkD = 0;

  

  // driving pid variables
  double tError = turnTarget - Inertial.heading();
  double prevError = 0;
  double integral = 0;
  double derivative = 0;
  double output = 0;

  if (tError>180) {
      tError-=360;
    }
    if (tError<-180) {
      tError+=360;
    }



  // Turning Pid
  while (fabs(tError) > turnAccuracy) {
    tError = turnTarget - Inertial.heading();

    output = (tkP * tError + tkI * integral + tkD * derivative);
    //output = limiter(output, -12, 12); // limits the volt for the output

    Brain.Screen.print(Inertial.heading());
    rDrive.spin(forward, -output, volt);
    lDrive.spin(forward, output, volt);

    

    integral += tError; // not used currently
    derivative = tError - prevError;
    prevError = tError;
    wait(10, msec);
  }
}


void autonomous(void) {
  // Long Side auton
  
  intake.spin(fwd, 12, volt);
  lDrive.spin(reverse, 6, volt);
  rDrive.spin(reverse, 6, volt);
  wait(350,msec);
  lDrive.stop();
  rDrive.stop();
  wait(100, msec);
  intake.stop();
  
  /*
  driveStraight(1440, .5);
  wait(150, msec);

  lDrive.spin(fwd, 3, volt);
  rDrive.spin(fwd, -3, volt);
  wait(715, msec);
  lDrive.stop();
  rDrive.stop();
  wait(150, msec);

  driveStraight(2520, .5);
  wait(150, msec);

  punch.spin(reverse, 12, volt);
  wait(2, sec);
  punch.stop();
  */

  //  Short side auton
  /*
  driveStraight(1080, .5);
  wait(150, msec);

  lDrive.spin(fwd, 3, volt);
  rDrive.spin(fwd, -3, volt);
  wait(710, msec);
  lDrive.stop();
  rDrive.stop();
  wait(150, msec);
  
  driveStraight(1440, .5);
  wait(150, msec);

  lDrive.spin(fwd, -3, volt);
  rDrive.spin(fwd, 3, volt);
  wait(705, msec);
  lDrive.stop();
  rDrive.stop();
  wait(150, msec);
  */   /*
  intake.spin(fwd, 12, volt);
  driveStraight(-1440, .5);
  wait(500, msec);
  intake.stop();
  wait(100, msec);

  driveStraight(1800, .5);
  wait(100, msec);
  lDrive.spin(fwd, -3, volt);
  rDrive.spin(fwd, 3, volt);
  wait(715, msec);
  lDrive.stop();
  rDrive.stop();
  wait(150, msec);

  driveStraight(2520, .5);
  wait(150, msec);
  punch.spin(reverse, 12, volt);
  wait(2, sec);
  punch.stop();
  */

  // Programing Skills
  /*
  intake.spin(forward, 12, volt);
  driveStraight(-1080, .5);
  wait(450, msec);
  intake.stop();
  wait(150, msec);

  driveStraight(1800, .5);
  wait(150, msec);

  lDrive.spin(fwd, 3, volt);
  rDrive.spin(fwd, -3, volt);
  wait(765, msec);
  lDrive.stop();
  rDrive.stop();
  wait(150, msec);
  */
  /*
  intake.spin(reverse, 12, volt);
  driveStraight(-1800, .5);
  wait(2, sec);
  intake.stop();
  wait(150, msec);
  */
  /*
  lDrive.spin(fwd, 6, volt);
  rDrive.spin(fwd, 6, volt);
  wait(1150, msec);
  lDrive.stop();
  rDrive.stop();
  wait(100, msec);

  lDrive.spin(fwd, -3, volt);
  rDrive.spin(fwd, 3, volt);
  wait(705, msec);
  lDrive.stop();
  rDrive.stop();
  wait(100, msec);

  lDrive.spin(fwd, 8, volt);
  rDrive.spin(fwd, 8, volt);
  wait(1000, msec);
  lDrive.stop();
  rDrive.stop();
  wait(100, msec);

  punch.spin(reverse, 12, volt);
  wait(2500, msec);
  punch.stop();
  */
  //
}

int largestSize = 0;
int largestIndex = 0; // variable to store the index of the largest object

void usercontrol(void) {

  while (true) {
    /*
    if(Controller1.ButtonA.pressing()){
      turningPid(90, .5);
    }
    */
    /*
    if(Controller1.ButtonY.pressing()){
      driveStraight(-2160, .5);
    }
    */
    if (Controller1.ButtonR2.pressing()) {punch.spin(reverse, 12, volt);}
    else if (LineTrackerH.reflectivity() <= 40 && Controller1.ButtonR2.pressing()) {punch.spin(reverse, 3, volt);}
    else if (LineTrackerH.reflectivity() <= 80 && !Controller1.ButtonR2.pressing()) {punch.spin(reverse, 6, volt);}
    else {punch.stop();}
  
    if( (Controller1.ButtonDown.pressing() && Controller1.ButtonB.pressing()) || (Controller1.ButtonB.pressing() && Controller1.ButtonDown.pressing()) ){
      piston.set(true);
    } else {
      piston.set(false);
    }
  

    if (Controller1.ButtonL2.pressing()&&Controller1.ButtonL1.pressing()) {intake.spin(reverse, 12, volt);}
    else if (Controller1.ButtonL2.pressing()) {intake.spin(reverse, 12, volt);}
    else if (Controller1.ButtonL1.pressing()) {intake.spin(forward, 12, volt);}
    else {intake.stop();}

    Vision14.takeSnapshot(Vision14__SIG_1);  //Take a picture
    Vision17.takeSnapshot(Vision17__SIG_1);  //Take a picture

    if(!Vision17.largestObject.exists){Vision17.takeSnapshot(Vision17__SIG_2);}
    //If it didn't see anything, take a picture looking for something different.

    for (int i = 0; i <= Vision14.objectCount; i++) {
      //Our FOV constraints
      if (Vision14.objects[i].centerX < 170 && Vision14.objects[i].centerX > 140) {
          //Makes sure area is greater than 20 or watever i set it lmao
          if ((Vision14.objects[i].width * Vision14.objects[i].height) > 100) {
            if ((Vision14.objects[i].width / Vision14.objects[i].height) >= 2.5) {

            // Get the size of the current object
            int currentSize = Vision14.objects[i].width * Vision14.objects[i].height;

            // If the current object is larger than the largest object, update the variable
            if (currentSize > largestSize) {
              largestSize = currentSize;
              largestIndex = i;
            }
          }
        }
      }
    }
    /*
    for (int i = 0; i <= Vision17.objectCount; i++) {
      //Our FOV constraints
            // Get the size of the current object
            int currentSize = Vision17.objects[i].width * Vision17.objects[i].height;
            // If the current object is larger than the largest object, update the variable
            if (currentSize > largestSize) {
              largestSize = currentSize;
              largestIndex = i;
            }  
    }
    int aimbotTarget = Vision14.objects[largestIndex].centerX+6;
    if (Controller1.ButtonX.pressing()) {
      aimbotTarget = Vision14.objects[largestIndex].centerX+6;
    } else if (Controller1.ButtonA.pressing()) {
      aimbotTarget = Vision17.objects[largestIndex].centerX;
    }
    */
    
    //Pid TIMEEE
    //Sensor in is Vision.largestobject.centerX, which returns the horizontal center
    err=160-(Vision14.objects[largestIndex].centerX+6);  //165 is my desired value.  
    speed=err-lasterr;
    lasterr=err;
    pidout=err*.04+speed*.05;  //I directly set my kp and kd without variables.

    if (!Controller1.ButtonX.pressing()||!Controller1.ButtonA.pressing()||!Vision14.largestObject.exists) {pidout=0;}
    drive=Controller1.Axis3.position()*.10;
    turn=Controller1.Axis1.position()*.10*.65 - pidout;//add pidout
    rDrive.spin(fwd, drive-turn, volt);
    lDrive.spin(fwd, drive+turn, volt);
    wait(30, msec);
  }
}
int main() { 
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  
  while(true) {
    wait(100, msec);
  }
} 
