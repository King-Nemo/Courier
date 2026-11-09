#pragma once
#include "vex.h" // IWYU pragma: keep

struct Point {
            float x;
            float y;
            float theta = std::numeric_limits<float>::quiet_NaN();

        // Constructor
            Point(float x, float y, float theta = std::numeric_limits<float>::quiet_NaN())
                : x(x),
                y(y),
                theta(theta) {}
        };


struct Coordinate{
    float x ;
    float y ;
};

struct Pose{

    float angle;
    Coordinate pos;
};

        
struct chassis{
        vex::inertial* imu;
        vex::motor_group* leftMotors;
        vex::motor_group* rightMotors;
        vex::rotation* horiz;
        vex::rotation* vert;

        chassis(vex::inertial* val, vex::motor_group* val1, vex::motor_group* val2, vex::rotation* val3, vex::rotation* val4) : imu(val), leftMotors(val1), rightMotors(val2), horiz(val3), vert(val4) {}
    };

