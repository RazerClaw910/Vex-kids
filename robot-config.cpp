#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

// Constructors for devices

controller Controller1 = controller(primary);

motor FR = motor(PORT6, ratio6_1, false);
motor BR = motor(PORT1, ratio6_1, false);
motor TR = motor(PORT10, ratio6_1, false);
motor_group rDrive = motor_group(FR, BR, TR);


motor FL = motor(PORT16, ratio6_1, true);
motor BL = motor(PORT11, ratio6_1, true);
motor TL = motor(PORT20, ratio6_1, true);
motor_group lDrive = motor_group(FL, BL, TL);


motor intake = motor(PORT21, ratio18_1, false);
motor puncher = motor(PORT9, ratio18_1, false);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}