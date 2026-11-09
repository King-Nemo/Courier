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
    bool reversed;
    bool steady;
    bool straight;
    bool cosScalling;
    float settleDist;
    float dlead;
    float MAXSPEED;
    int settleLength;
    int timeout = 100000;
    void init(){
        reversed = false;
        steady = false;
        straight = false;
        cosScalling = true;
        settleDist = 0;
        dlead = 0.1;
        MAXSPEED = 0;
        settleLength = 10;
        timeout = 100000;
    }
};


void initMotion(chassis* initC, std::vector<float> angV = {0.01,0,12}, std::vector<float> latV = {3,0,6});

std::vector<double> toPointStep(float sigX,float sigY, std::vector<float> pos);
void toPoint(float tarX, float tarY, float exit, moveParams params = {});

void toPose(float tarX, float tarY, float tarT, float exit, moveParams params = {});

void toAng(float tarT, float exit);

void turnToPoint(float sigX, float sigY, float exit);

void arcToDist(float dist, float exit, float leftSpeed = 127, float rightSpeed = 127, moveParams params = {});

