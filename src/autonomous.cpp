#include "vex.h";
#include "odometery.hpp";
#include "motion.hpp";
#include "util.hpp";
#include "autonomous.hpp";

void testAuton(){
    setOdom(0, 0);
    toDist(-10,1,3);
    //driveTo(1.7,3000);
    lift.spin(reverse, 10, volt);
    wait(500, msec);
    //claw.set(false);
    lift.stop();
    wait(500, msec);
    toDist(6.5,1,3);
    toAng(85,1);
    toDist(-7,-1, 3, moveParams().setMaxSpeed(2.5));
    
    //claw.set(true);
    wait(500, msec);
    lift.spinFor(forward, 280, degrees, false);
    toAng(-25,1000);
    // move To point (0,-10) for other goal ;-;
    // moveToPoint(0,-10,1,3000);
    toDist(-8.2,-1,3);
    lift.spin(reverse, 8, volt);
    wait(500, msec);
    //claw.set(false);
    
    wait(500, msec);
    lift.stop();
    toDist(8,3,1);
    toAng(15,1000);
    toDist(-5,3000, false);
    // swingpower(-9,-3, -135,2000);
    // turnToAngle(-135,500);
    // // Toggle
    // driveTo(5,3000, true);
    // driveTo(-15,1000);
    // driveTo(7,3000, true);
    // wait(500, msec);
    // driveTo(-9,1000, true);
    // //wait(500, msec);
    // driveTo(7,1000, true);

}