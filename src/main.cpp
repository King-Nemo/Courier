/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Owen  C (2668A)                                           */
/*    Created:      8/23/2026, 4:45:48 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#pragma once
#include "vex.h"
#include "odometery.hpp"
#include "motion.hpp"
#include "util.hpp"
#include "autonomous.hpp"

// A global instance of competition
vex::competition Competition;

// define your global instances of motors and other devices here


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  driveParams params(&inertial_sensor, &right_chassis, &left_chassis, &horizontal_tracker);
  // params.sV_in = 9.9/2;
  params.sV_in = 0.0; // bc im running both sides of the drive
  params.sS_in = horizontal_tracker_dist_from_center;
  params.YwheelDiameter = (1/M_PI);
  params.XwheelDiameter = horizontal_tracker_diameter;
  params.DriveRatio = wheel_distance_in;

  horizontal_tracker.resetPosition();
  inertial_sensor.calibrate();
  while(inertial_sensor.isCalibrating()) vex::wait(10, vex::msec);
  
  init_odom(DRIVEX, params);
  // User control code here, inside the loop
  chassis mainChassis(&inertial_sensor, &left_chassis, &right_chassis, &horizontal_tracker, nullptr);
  initMotion(&mainChassis);

  setOdom(0, 0);

  Brain.Screen.print("X: %f Y: %f Theta %f", getPos()[0], getPos()[1], getPos()[2]);

  //vex::wait(5000, vex::msec);
  
  // toPoint(15, 15, 2);
  // wait(2000, vex::msec);
  // toDist(15, -1, 2);
  // wait(200, vex::msec);
  // toAng(90, 5);
  // wait(200, vex::msec);
  //turnToPoint(0, 0, 5);

  testAuton();

  while (true) {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("X: %f Y: %f Theta %f", getPos()[0], getPos()[1], getPos()[2]);
    vex::wait(20, vex::msec);
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    vex::wait(100, vex::msec);
  }
}
