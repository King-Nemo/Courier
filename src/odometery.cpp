#pragma once
#include "odometery.hpp"
#include "vex.h"
#include "util.hpp"


std::vector<std::vector<double>> fieldMap = {{0,0},{144,0},{144,144},{0,144},{0,0}};
std::vector<particle> particles;

float prevAngle = 0;
float prevHoriz = 0;
float prevVert = 0;

float thetaDelta = 0;
float vertDelta = 0;
float horizDelta = 0;

float theta;
float vert;
float horiz;

float xPos = 0;
float yPos = 0;
float sV;
float sS;

float MCLParticles = 144;
float sigma = 1.0;
std::vector<MCLsensor> MCLSensors;

float deltaX;
float deltaY;

std::vector<float> DeltaCoords;

std::random_device rd;

std::mt19937 gen(rd());

void updateDeltas(){
  thetaDelta = theta - prevAngle;
  vertDelta = vert - prevVert;
  horizDelta = horiz - prevHoriz;
}

void updatePrev(){
  prevAngle = theta;
  prevVert = vert;
  prevHoriz = horiz;
}

void odomCalc(){
  //  thetaDelta Will need to be in radians

  float rot;
  

  //float angleDelta = (leftDelta - rightDelta) / (sL + sR); // if we lacked a IMU we could use this

  float angleDelta = thetaDelta;

  if (std::abs(angleDelta) > 0.0174533){

    DeltaCoords = {2*std::sin(angleDelta/2)*((horizDelta/angleDelta) + sS), 2*std::sin(angleDelta/2)*((vertDelta/angleDelta) + sV)};

  }else{
    DeltaCoords = {horizDelta, vertDelta}; 
  }
  
  rot = prevAngle + (angleDelta/2);
  //rot = rot*(180/M_PI);
  deltaX = DeltaCoords[0]*std::cos(rot) - DeltaCoords[1]*std::sin(rot); 
  deltaY = DeltaCoords[0]*std::sin(rot) + DeltaCoords[1]*std::cos(rot);

  xPos = xPos + deltaX; 
  yPos = yPos + deltaY;
}

void mclInit(MCLParams* params){
  std::uniform_real_distribution<double> distr(0.0, 144.0);
  sigma = params->sigma;
  MCLParticles = params->MCLParticles;
  MCLSensors = params->sensors;
  for (int i = 0; i < MCLParticles; i++) {
    float xDist = distr(gen);
    float yDist = distr(gen);
    particles.push_back(particle(xDist, yDist, 1.0 / MCLParticles));
  }
}


void mclStep(){
  float inv_two_sigma_sq = 1.0 / (2.0 * sigma * sigma);
  std::normal_distribution<double> noise(0.0, sigma);
  std::uniform_real_distribution<double> wheelOfFortune(0.0, 1.0 / MCLParticles);
  float maxReading = 78.75;
  std::vector<MCLreading> frame;
  for (const auto& sensor : MCLSensors){
    float reading = sensor.sensor->objectDistance(vex::mm);
    float offset = sensor.offset;
    float turn = sensor.turn;
    frame.push_back(MCLreading(reading, offset, turn));
  }
  for (auto& p : particles) {
    p.weight = 1.0;
    // move each particle by odom deltas along with noise and set heading to theta
    p.x += deltaX + noise(gen);
    p.y += deltaY + noise(gen);

    // simulate readings for each particle and then compare them to actual readings
    // weight each particle (1/sigma*sqrt(2*pi)*e^-((reading - actual)^2/(2*sigma^2)))

    // NEED TO ACCOUNT FOR OFFSET OF SENSORS
    for (const auto& read : frame){
      float min_distance = maxReading;
      float localTheta = theta + read.turn;
      for (int i = 0; i < fieldMap.size()-1; i++){
        float x1 = fieldMap[i][0];
        float y1 = fieldMap[i][1];
        float x2 = fieldMap[i+1][0];
        float y2 = fieldMap[i+1][1];

        float ry = std::sin(localTheta); 
        float rx = std::cos(localTheta);

        if (std::min(x1, x2) > p.x + maxReading && rx > 0) {
          continue;
        }

        float denom = (x1 - x2) * ry - (y1 - y2) * rx;
        if (std::abs(denom) < 1e-6){
          continue;
        }
        float t = ((x1 - p.x) * ry - (y1 - p.y) * rx) / denom;
        float u = ((x1 - x2) * (y1 - p.y) - (y1 - y2) * (x1 - p.x)) / denom;

        if (0 <= t && t <= 1 && u >= 0) {
            if (u < min_distance) {
                min_distance = u;
            }
        }
      }
      
      if (min_distance > 54){
        continue;
      }
      // Weight the particle based on the minimum distance
      float particleError = min_distance - read.reading;
      float weightHold = (1.0 / (min_distance * sqrt(2 * M_PI) * sigma)) * exp(-1 * (particleError * particleError) * inv_two_sigma_sq);
      p.weight *= weightHold;
      
      
    }

  }
  // Resample particles based on weight using Stochastic Universal Sampling
  float particleSum = 0;
  for (auto& p : particles){
    particleSum += p.weight;
  }
  for (auto& p : particles){
    p.weight /= particleSum;
  }
  float step = 1.0 / MCLParticles;
  float r = wheelOfFortune(gen);
  float c = particles[0].weight;
  int i = 0;
  std::vector<particle> newParticles;
  for (int m = 0; m < MCLParticles; m++) {
    float U = r + m * step;
    while (U > c) {
      i++;
      c += particles[i].weight;
    }
    newParticles.push_back(particles[i]);
  }
  particles = newParticles;
}



void odomDrive(void* param){
initParams* params = static_cast<initParams*>(param);
// sV_in, float sS_in, int imu_port, int tracking_port

sS = params->sS_in;
sV = params->sV_in;
float diameter = params->YwheelDiameter;
float driveRatio = params->DriveRatio;

vex::inertial* imu = (params->imu);
vex::motor_group* vert_m = (params->driveMotor);


int go = 1;
while (go==1){
  horiz = 0;
  vert = vert_m->position(vex::deg) * (diameter * M_PI) * driveRatio / 360;// I used 4 inch wheels, so the 4 would be changed to what every size wheels And the 3/5 is the gear ratio
  theta = imu->heading() * (M_PI/180); // gets the inertial and converts to radians
  updateDeltas();
  odomCalc();
  updatePrev();
  if (params->mcl == true){
    mclStep();
  }

  vex::task::sleep(10);
  //pros::delay(10);
}
}

void odomY(void* param){
odomParams* params = static_cast<odomParams*>(param);
// sV_in, float sS_in, int imu_port, int tracking_port

sS = params->sS_in;
sV = params->sV_in;
float diameter = params->YwheelDiameter;

vex::inertial* imu = (params->imu);
vex::rotation* vert_r = (params->vert);


int go = 1;
while (go==1){
  horiz = 0;
  vert = (vert_r->position(vex::deg) * (diameter * M_PI)/ 360) / 100;// I used 4 inch wheels, so the 4 would be changed to what every size wheels And the 3/5 is the gear ratio
  theta = imu->heading() * (M_PI/180); // gets the inertial and converts to radians
  updateDeltas();
  odomCalc();
  updatePrev();

  vex::task::sleep(10);
}
}

void odomX(void* param){
initParams* params = static_cast<initParams*>(param);
// sV_in, float sS_in, int imu_port, int tracking_port

sS = params->sS_in;
sV = params->sV_in;
float Xdiameter = params->XwheelDiameter;
float Ydiameter = params->YwheelDiameter;
float driveRatio = params->DriveRatio;


vex::motor_group* vert_m = (params->driveMotor);
vex::inertial* imu = (params->imu);
vex::rotation* horiz_m = (params->horiz);


int go = 1;
while (go==1){
  horiz = (horiz_m->position(vex::deg) * (Xdiameter * M_PI)/ 360)/100; //This is temporary becuase I dont feel like seting up a traking wheel, and dont have a rotation sensor
  vert = (vert_m->position(vex::deg) * (Ydiameter * M_PI) * driveRatio / 360);// I used 4 inch wheels, so the 4 would be changed to what every size wheels And the 3/5 is the gear ratio
  theta = imu->heading() * (M_PI/180); // gets the inertial and converts to radians
  updateDeltas();
  odomCalc();
  updatePrev();

  vex::task::sleep(10);
}
}

void odomXY(void* param){
odomParams* params = static_cast<odomParams*>(param);
// sV_in, float sS_in, int imu_port, int tracking_port

sS = params->sS_in;
sV = params->sV_in;
float YwheelDiameter = params->YwheelDiameter;
float XwheelDiameter = params->XwheelDiameter;

vex::inertial* imu = (params->imu);
vex::rotation* horiz_m = (params->horiz);
vex::rotation* vert_m = (params->vert);


int go = 1;
while (go==1){
  horiz = (horiz_m->position(vex::deg) * (XwheelDiameter * M_PI) / 360) / 100;
  vert = (vert_m->position(vex::deg) * (YwheelDiameter * M_PI) / 360) / 100;// I used 4 inch wheels, so the 4 would be changed to what every size wheels And the 3/5 is the gear ratio
  theta = imu->heading() * (M_PI/180); // gets the inertial and converts to radians
  updateDeltas();
  odomCalc();
  updatePrev();

  vex::task::sleep(10);
}
}


//functions to initiallize the odom system

bool init_odom(enum config con, initParams params){
  initParams* heap_params = new initParams(params);

  if (con == DRIVE){
    //setup for drive only
    vex::thread odo(odomDrive, heap_params);
  }else if (con == XTRACK){
    //setup for xtrack only
    vex::thread odo(odomX, heap_params);
  }else{
    delete heap_params;
    return false;
  }
  return true;

}

bool init_odom(enum config con, odomParams params){
  odomParams* heap_params = new odomParams(params);

  if (con == YTRACK){
    //setup for ytrack only
    vex::thread odo(odomY, heap_params);
  }else if (con == XYTRACK){
    //setup for xytrack
    vex::thread odo(odomXY, heap_params);
  }else{
    delete heap_params;
    return false;
  }
  return true;

}


bool init_odom(enum config con, chassis chass, TaskParams params){
  TaskParams* heap_params = new TaskParams(params);
  chassis* heap_chass = new chassis(chass);

  if (con == DRIVE){
    //setup for drive only
    initParams* init_params = new initParams(heap_chass->imu, heap_chass->leftMotors, heap_chass->horiz);
    vex::thread odo(odomDrive, heap_params);
  }else if (con == XTRACK){
    //setup for xtrack only
    vex::thread odo(odomX, heap_params);
  }else if (con == YTRACK){
    //setup for ytrack only
    vex::thread odo(odomY, heap_params);
  }else if (con == XYTRACK){
    //setup for xytrack
    vex::thread odo(odomXY, heap_params);
  }else{
    delete heap_params;
    return false;
  }
  return true;

}

std::vector<double> getPos(){
  return {-xPos, yPos, theta * (180/M_PI)};
}

float getAng(){
  return theta * (180/M_PI);
}

std::vector<double> getVals(){
  return {vert, horiz, theta * (180/M_PI)};  
}

void setOdom(float x, float y){
  xPos = -x;
  yPos = y;
}

