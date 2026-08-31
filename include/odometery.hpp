#pragma once
#include "vex.h"


enum config {
    DRIVE,
    XTRACK,
    YTRACK,
    XYTRACK
};


struct TaskParams {
        float sV_in;
        float sS_in;
        int imu_port;
        int vert_port;
        int horiz_port;
        float YwheelDiameter;
        float XwheelDiameter;
        float DriveRatio;
    };

struct initParams {
        float sV_in;
        float sS_in;
        float YwheelDiameter;
        float XwheelDiameter;
        float DriveRatio;
        vex::inertial* imu;
        vex::motor_group* driveMotor;
        vex::rotation* horiz;
        bool mcl = false;

        initParams(vex::inertial* val, vex::motor_group* val1, vex::rotation* val2) : imu(val), driveMotor(val1), horiz(val2) {}
    };

struct odomParams {
        float sV_in;
        float sS_in;
        float YwheelDiameter;
        float XwheelDiameter;
        float DriveRatio;
        vex::inertial* imu;
        vex::rotation* vert;
        vex::rotation* horiz;

        odomParams(vex::inertial* val, vex::rotation* val1, vex::rotation* val2) : imu(val), vert(val1), horiz(val2) {}
    };

struct MCLsensor{
        vex::distance* sensor;
        float offset;
        float turn;
        MCLsensor(vex::distance* val, float val1, float val2) : sensor(val), offset(val1), turn(val2) {}
    };

struct MCLParams{
        float sigma;
        float MCLParticles;
        std::vector<MCLsensor> sensors;
};

struct MCLreading{
        float reading;
        float offset;
        float turn;
        MCLreading(float val, float val1, float val2) : reading(val), offset(val1), turn(val2) {}
    };

class particle{
    public:
        float x;
        float y;
        float weight;
        particle(float x_in, float y_in, float weight_in){
            x = x_in;
            y = y_in;
            weight = weight_in;
        }
};


void odomCalc();
void updateDeltas();
void updatePrev();
void odomDrive(void* param);
void odomX(void* param);
void odomY(void* param);
void odomXY(void* param);
bool init_odom(enum config con, initParams params);
bool init_odom(enum config con, odomParams params);
bool init_odom(enum config con, float sV_in, int imu_port, int tracking_port, float wheelDiameter);
bool init_odom(enum config con, float sV_in, float sS_in, int imu_port, int horiz_port, int vert_port, float YwheelDiameter, float XwheelDiameter);
std::vector<double> getPos();
std::vector<double> getVals();

float getAng();
void setOdom(float x, float y);



