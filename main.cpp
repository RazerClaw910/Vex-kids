/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\Razer                                            */
/*    Created:      Sun Dec 18 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;
competition Competition;

void preAuton() {
  vexcodeInit();
}

void autonomous(void) {
  intake.spin(reverse, 12, volt);
  wait(450, msec);
  intake.stop();
}

void usercontrol(void) {
  double deadzoneY;
  float drive;
  float turn;
  while(true) {
    // Driving
    if (Controller1.Axis3.position() < -40.625) {
      deadzoneY = -40.625;
    } else {
      deadzoneY = Controller1.Axis3.position();
    }
    drive = deadzoneY*.2;
    turn = Controller1.Axis1.position()*.2*.65;
    rDrive.spin(fwd, drive-turn, volt);
    lDrive.spin(fwd, drive+turn, volt);

    // intake
    if (Controller1.ButtonL1.pressing()) {
      intake.spin(forward, 12, volt);
    } else if (Controller1.ButtonL2.pressing()) {
      intake.spin(reverse, 12, volt);
    } else {
      intake.stop();
    }

    // puncher
    if (Controller1.ButtonR2.pressing()) {
      puncher.spin(reverse, 12, volt);
    } else {
      puncher.stop();
    }
  }  
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  preAuton();
  while(true) {
    wait(100, msec);
  }
}
