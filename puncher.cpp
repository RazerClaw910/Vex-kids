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
// Vision14             vision        7               
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


controller Controller1 = controller(primary);
motor BR = motor(PORT1, ratio6_1, false);
 
motor FR = motor(PORT6, ratio6_1, false);
 
motor BL = motor(PORT11, ratio6_1, true);
 
motor FL = motor(PORT16, ratio6_1, true);
 
motor TR = motor(PORT10, ratio6_1, false);
 
motor TL = motor(PORT20, ratio6_1, true);
 
motor intake = motor(PORT21, ratio18_1, true);
 
motor punch = motor(PORT9, ratio18_1, false);

//intake.setVelocity(100,percent);
float drive;
float turn;
//wait(500,msec);
int err;
int lasterr=0;
int pidout;
int speed=0;

void autonomous(void) {
  //float driveAuton = 200;
  //float turnAuton = 0;
  intake.spin(forward, 50, percent);
  wait(1, seconds);
  intake.stop();
  /*
  FR.spin(forward,driveAuton-turnAuton,volt);
  FL.spin(forward,driveAuton-turnAuton,volt);
  wait(5000, msec);
  FR.stop();
  FL.stop();
  */
  }

void usercontrol(void) {
  while (true) {
    // if button is pressed, run motor for 1 turn, set primed to false.
    // if primed is false run motor
    //if power>1 watt set primed to true and cat.spinfor(2.5 rotations)
    //
    //
    //Once button is released, run motor. if motor isrun motor

    //if (Controller1.ButtonB.pressing() && Controller1.ButtonDown.pressing() {})

    if (Controller1.ButtonR2.pressing()) {punch.spin(reverse, 12, volt);}
    else {punch.stop();}
  

    if (Controller1.ButtonL2.pressing()&&Controller1.ButtonL1.pressing()) {intake.spin(reverse, 12, volt);}
    else if (Controller1.ButtonL2.pressing()) {intake.spin(reverse, 12, volt);}
    else if (Controller1.ButtonL1.pressing()) {intake.spin(forward, 12, volt);}
    else {intake.stop();}

    Vision14.takeSnapshot(Vision14__SIG_1);  //Take a picture
    //If it didn't see anything, take a picture looking for something different.
    if (!Vision14.largestObject.exists) {Vision14.takeSnapshot(Vision14__SIG_2);} 
    
    //Pid TIMEEE
    //Sensor in is Vision.largestobject.centerX, which returns the horizontal center
    err=160-Vision14.largestObject.centerX;  //165 is my desired value.  
    speed=err-lasterr;
    lasterr=err;
    pidout=err*.08+speed*.12;  //I directly set my kp and kd without variables.

    if (!Controller1.ButtonX.pressing()||!Vision14.largestObject.exists) {pidout=0;}

    drive=Controller1.Axis3.position()*.12;
    turn=Controller1.Axis1.position()*.12*.45; //- pidout;//add pidout
    FR.spin(forward,drive-turn,volt);
    BR.spin(forward,drive-turn,volt);
    TR.spin(forward,drive-turn,volt);
    FL.spin(forward,drive+turn,volt);
    BL.spin(forward,drive+turn,volt);
    TL.spin(forward,drive+turn,volt);
}
}
int main() { 
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  
  while(true) {
    wait(100, msec);
  }
} 
