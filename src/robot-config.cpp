//credit to Richard wang (took this from RW cuz im lazy)

#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain = brain();

// VEXcode device constructors
controller controller_1 = controller(primary);

// IMPORTANT: Remember to modify the example motors according to the guide. 
// Also remember to add respective device declarations to custom/include/robot-config.h
// Format: motor(port, gearSetting, reversed)
// gearSetting is one of the following: ratio36_1(red), ratio18_1(green), ratio6_1(blue)
// all chassis motors should be reversed appropriately so that they spin vertical when given a positive voltage input
// such as driveChassis(12, 12)
motor left_chassis1 = motor(PORT11, ratio6_1, false);
motor left_chassis2 = motor(PORT12, ratio6_1, false);
motor_group right_chassis = motor_group(left_chassis1, left_chassis2);
motor right_chassis1 = motor(PORT13, ratio6_1, true);
motor right_chassis2 = motor(PORT14, ratio6_1, true);
motor_group left_chassis = motor_group(right_chassis1, right_chassis2);

inertial inertial_sensor = inertial(PORT6);


// Format is rotation(port, reversed)
// just set these to random ports if you don't use tracking wheels
rotation horizontal_tracker = rotation(PORT5, false);
rotation vertical_tracker = rotation(PORT8, false);

// game specific devices for high stakes
motor intaketop = motor(PORT4, ratio6_1, true);
motor intakebottom = motor(PORT3, ratio18_1, false);
motor_group intake = motor_group(intaketop,intakebottom);

motor leftlift = motor(PORT1, ratio36_1, false);
motor rightlift = motor(PORT2, ratio36_1, true);
motor_group lift = motor_group(leftlift,rightlift);

digital_out tray = digital_out(Brain.ThreeWirePort.A);
digital_out claw = digital_out(Brain.ThreeWirePort.C);
digital_out tilter = digital_out(Brain.ThreeWirePort.B);

//distance front = distance(PORT4);
//distance rear = distance(PORT5);

//optical wingsensor = optical(PORT14);







// ============================================================================
// USER-CONFIGURABLE PARAMETERS (CHANGE BEFORE USING THIS TEMPLATE)
// ============================================================================

// Distance between the middles of the left and right wheels of the drive (in inches)
double distance_between_wheels = 9.9;

// motor to wheel gear ratio * wheel diameter (in inches) * pi
double wheel_distance_in = (36.0 / 48.0) * 3.25 * M_PI;

// PID Constants for movement
// distance_* : Linear PID for strai ght driving
// turn_*     : PID for turning in place
// heading_correction_* : PID for heading correction during linear movement
double distance_kp = 1.1, distance_ki = 0.1, distance_kd = 15;
double turn_kp = 0.32, turn_ki = 0.02, turn_kd = 2.8;
double heading_correction_kp = 0.55, heading_correction_ki = 0, heading_correction_kd = 4;



// Enable or disable the use of tracking wheels
bool using_horizontal_tracker = true;  // Set to true if a horizontal tracking wheel is installed and used for odometry
bool using_vertical_tracker = false;   // Set to true if a vertical tracking wheel is installed and used for odometry

// IGNORE THESE IF YOU ARE NOT USING TRACKING WHEELS
// These comments are in the perspective of a top down view of the robot when the robot is facing vertical
// Vertical distance from the center of the bot to the horizontal tracking wheel (in inches, positive is when the wheel is behind the center)
double horizontal_tracker_dist_from_center = 4.5;
// Horizontal distance from the center of the bot to the vertical tracking wheel (in inches, positive is when the wheel is to the right of the center)
double vertical_tracker_dist_from_center = 0.2;
double horizontal_tracker_diameter = 2.74; // Diameter of the horizontal tracker wheel (in inches)
double vertical_tracker_diameter = 2.73; // Diameter of the vertical tracker wheel (in inches)




// ============================================================================
// ADVANCED TUNING (OPTIONAL)
// ============================================================================

bool heading_correction = true; // Use heading correction when the bot is stationary

// Set to true for more accuracy and smoothness, false for more speed
bool dir_change_start = true;   // Less accel/decel due to expecting direction change at start of movement
bool dir_change_end = true;     // Less accel/decel due to expecting direction change at end of movement

double min_output = 4; // Minimum output voltage to motors while chaining movements

// Maximum allowed change in voltage output per 10 msec during movement
double max_slew_accel_fwd = 20;
double max_slew_decel_fwd = 12;
double max_slew_accel_rev = 16;
double max_slew_decel_rev = 10;

// Prevents too much slipping during boomerang movements
// Decrease if there is too much drifting and inconsistency during boomerang
// Increase for more speed during boomerang
double chase_power = 1;












// ====================================+
// DO NOT CHANGE ANYTHING BELOW
// ============================================================================

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

 


















/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // nothing to initialize
}