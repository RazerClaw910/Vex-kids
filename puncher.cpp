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
/*
#define repeat(iterations)                                                     \\
 for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS
 */
 
// Robot configuration code.

 
 
 
/*vex-vision-config:begin*/
 
inertial Inertial1 = inertial(PORT13);
 
 
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
double drivetrainPos() {
  return (rDrive.position(degrees) + lDrive.position(degrees))/2;
}

double error;

void driveStraight(double target, double accuracy) {
  // Resets motor encoders
  rDrive.setPosition(0,degrees);
  lDrive.setPosition(0,degrees);

  // k values
  const double kP = 0.00245;
  const double kI = 0;
  const double kD = 0.029;

  

  // driving pid variables
  double prevError = 0;
  double integral = 0;
  double derivative = 0;
  double output = 0;



  // Driving pid
  while (true) {

    if (fabs(error) < accuracy) {
      break;
    }

    error = target - Inertial.heading();
    Brain.Screen.print(output);
    Brain.Screen.newLine();
    output = (kP * error + kI * integral + kD * derivative);
    output = limiter(output, -12, 12); // limits the volt for the output
    rDrive.spin(forward, output, volt);
    lDrive.spin(forward, output, volt);
    integral += error; // not used currently
    derivative = error - prevError;
    prevError = error;
    wait(10, msec);
  }
}


void turningPid(double turnAngle, double accuracy) {
  // Resets motor encoders
  rDrive.setPosition(0,degrees);
  lDrive.setPosition(0,degrees);

  // k values
  const double tkP = 0.045;
  const double tkI = 0;
  const double tkD = 0;

  // gets the current posistion of the drive train
  double currentPos = drivetrainPos();

  // driving pid variables
  double turnError = turnAngle - currentPos;
  double turnPrevError = 0;
  double turnIntegral = 0;
  double turnDerivative = 0;
  double turnOutput = 0;

  // Driving pid
  while (fabs(turnError) > accuracy) {
    currentPos = drivetrainPos();
    turnError = turnAngle - currentPos;
    Brain.Screen.print(turnOutput);
    Brain.Screen.newLine();
    turnOutput = (tkP * turnError + tkI * turnIntegral + tkD * turnDerivative);
    turnOutput = limiter(turnOutput, -12, 12); // limits the volt for the output 
    rDrive.spin(forward, turnOutput, volt);
    lDrive.spin(forward, turnOutput, volt);
    turnIntegral += turnError; // not used currently
    turnDerivative = turnError - turnPrevError;
    turnPrevError = turnError;
    wait(10, msec);
  }
}


void autonomous(void) {
  driveStraight(360, .5);
  /*
  rDrive.resetPosition();
  lDrive.resetPosition();
  lDrive.spin(fwd, 12, volt);
  rDrive.spin(fwd, 12, volt);  nbn  
  intake.spin(fwd, 12, volt);
  wait(1500, msec);
  intake.stop();
  driveStraight(-1080, .5);
  rDrive.spin(fwd, -4, volt);
  lDrive.spin(fwd, 4, volt);
  driveStraight(1800, .5);
  punch.spin(reverse, 12, volt);
  wait(1, sec);
  punch.stop();
  */
  /*
  float driveAuton = 200;
  float turnAuton = 0;
  
  intake.spin(forward, 100, percent);
  //for (int i = 0; i < 2; i++) {
  FR.spin(reverse,driveAuton-turnAuton,volt);
  FL.spin(reverse,driveAuton-turnAuton,volt);
  wait(375, msec);
  FR.stop();
  FL.stop();
    //FR.spin(forward,driveAuton-turnAuton,volt);
    //FL.spin(forward,driveAuton-turnAuton,volt);
    //wait(1000, msec);
    //FR.stop();
    //FL.stop();
    //}
  intake.stop();
  */
  }

int largestSize = 0;
int largestIndex = 0; // variable to store the index of the largest object

void usercontrol(void) {
  /*
  punch.setVelocity(100,percent);
  punch.spin(reverse);
  punch.setStopping(brake);
  wait(500,msec);
  */
  while (true) {
    // if button is pressed, run motor for 1 turn, set primed to false.
    // if primed is false run motor
    //if power>1 watt set primed to true and cat.spinfor(2.5 rotations)
    //
    //
    //Once button is released, run motor. if motor isrun motor

    if (Controller1.ButtonR2.pressing()) {punch.spin(reverse, 12, volt);}
    else if (LineTrackerH.reflectivity() <= 99 && !Controller1.ButtonR2.pressing()) {punch.spin(reverse, 3, volt);}
    else {punch.stop();}
  
    if(Controller1.ButtonDown.pressing() && Controller1.ButtonB.pressing()){
      piston.set(true);
    }
  

    if (Controller1.ButtonL2.pressing()&&Controller1.ButtonL1.pressing()) {intake.spin(reverse, 12, volt);}
    else if (Controller1.ButtonL2.pressing()) {intake.spin(reverse, 12, volt);}
    else if (Controller1.ButtonL1.pressing()) {intake.spin(forward, 12, volt);}
    else {intake.stop();}

    Vision14.takeSnapshot(Vision14__SIG_1);  //Take a picture
    //If it didn't see anything, take a picture looking for something different.
    if (!Vision14.largestObject.exists) {Vision14.takeSnapshot(Vision14__SIG_2);} 

    for (int i = 0; i < Vision14.objectCount; i++) {
      //Our FOV constraints
      if (Vision14.objects[i].centerX < 170 && Vision14.objects[i].centerX > 140 && Vision14.objects[i].centerY < 150) {
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
    
    //Pid TIMEEE
    //Sensor in is Vision.largestobject.centerX, which returns the horizontal center
    err=160-((Vision14.objects[largestIndex].centerX)+6);  //165 is my desired value.  
    speed=err-lasterr;
    lasterr=err;
    pidout=err*.04+speed*.06;  //I directly set my kp and kd without variables.

    if (!Controller1.ButtonX.pressing()||!Vision14.largestObject.exists) {pidout=0;}
    drive=Controller1.Axis3.position()*.10;
    turn=Controller1.Axis1.position()*.10*.65 - pidout;//add pidout
    FR.spin(forward,drive-turn,volt);
    BR.spin(forward,drive-turn,volt);
    TR.spin(forward,drive-turn,volt);
    FL.spin(forward,drive+turn,volt);
    BL.spin(forward,drive+turn,volt);
    TL.spin(forward,drive+turn,volt);
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
