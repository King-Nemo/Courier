#pragma once
#include "vex.h"
#include "util.hpp"
#include "odometery.hpp"
#include <iostream> 
#include <cstdlib> // For integer abs()
#include <cmath>
#include <utility>
#include <algorithm> 
#include <vector>


class PID{
    public:
    float kP;
    float kI;
    float kD;
    float integral = 0;
    int start = 0;
    float prevError = 0;
    float out = 0;
    float derivative;
    float output;
    PID(float inkP, float inkI, float inkD);
    void changeVals(float inkP, float inkI, float inkD);
    float update(float sig);
    float update(float sig, float pos);
    float update(float sig, bool EMA, float alpha);
    void reset();
    
};

struct moveParams{
    bool reversed = false;
    bool steady = false;
    bool straight = false;
    bool cosScalling = false;
    float settleDist = 0;
    float dlead = 0.1;
    float MAXSPEED = 0;
    int settleLength = 10;
    int timeout = 100000;
    float angAccelSlew = max_slew_accel_ang;
    float latAccelSlew = max_slew_accel_fwd;
    float angDecelSlew = max_slew_decel_ang;
    float latDecelSlew = max_slew_decel_fwd;
    float angleExit = 0;
    // void init(){
    //     reversed = false;
    //     steady = false;
    //     straight = false;
    //     cosScalling = false;
    //     settleDist = 0;
    //     dlead = 0.1;
    //     MAXSPEED = 0;
    //     settleLength = 10;
    //     timeout = 100000;
    //     angAccelSlew = max_slew_accel_ang;
    //     latAccelSlew = max_slew_accel_fwd;
    //     angDecelSlew = max_slew_decel_ang;
    //     latDecelSlew = max_slew_decel_fwd;
    // }

    moveParams& setReversed(bool v) { reversed = v; return *this; }
    moveParams& setMaxSpeed(float v) { MAXSPEED = v; return *this; }
    moveParams& setTimeout(int v) { timeout = v; return *this; }
    moveParams& setStraight(bool v) { straight = v; return *this; }
    moveParams& setCosineScaling(bool v) { cosScalling = v; return *this; }
    moveParams& setSettleLength(int v) { settleLength = v; return *this; }
    moveParams& setDlead(float v) { dlead = v; return *this; }
    moveParams& setSettleDist(float v) { settleDist = v; return *this; }
};


void initMotion(chassis* initC, std::vector<float> angV, std::vector<float> latV);

void initMotion(chassis* intiC);

std::vector<double> toPointStep(float sigX,float sigY, std::vector<float> pos);
void toPoint(float tarX, float tarY, float exit, moveParams params = {});

void toPose(float tarX, float tarY, float tarT, float exit, moveParams params = {});

void toAng(float tarT, float exit, moveParams params = {});

void turnToPoint(float sigX, float sigY, float exit, moveParams params = {});

void toDist(float tarDist, int dir, float exit, moveParams params = {});

void arcToDist(float dist, float exit, float leftSpeed = 127, float rightSpeed = 127, moveParams params = {});

