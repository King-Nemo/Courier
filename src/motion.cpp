#include "motion.hpp"
#include <cmath>


float right_out = 0;
float left_out = 0;
float dist;



int sgn(float val){
  if (val > 0){
    return 1;
  }else if(val < 0){
    return -1;
  }else{
    return 0;
  }
}

double constrainAngle(double x){
    x = fmod(x + 180,360);
    if (x < 0)
        x += 360;
    return x - 180;
}

double constrain90(double x){
    x = fmod(x + 90,180);
    if (x < 0)
        x += 180;
    return x - 90;
}

float outA;
float outL; 


PID::PID(float inkP, float inkI, float inkD){
        kP = inkP;
        kI = inkI;
        kD = inkD;
}

void PID::changeVals(float inkP, float inkI, float inkD){

        kP = inkP;
        kI = inkI;
        kD = inkD;
}

    float PID::update(float sig){
      
        float error = sig;
        
        if (start == 0){
          prevError = error;
          start = 1;
        }
        integral += error;
        derivative = error - prevError;
        output =  kP * error + kI * integral + kD * derivative;
        prevError = error;

        if(sgn(error) != sgn(prevError)) integral = 0;

        if(std::abs(error) < 0.5) integral = 0;
        
        // Brain.Screen.clearScreen();
        // Brain.Screen.setCursor(1, 1);
        // Brain.Screen.print("err: %f out: %f deriv %f", sig, output, kP);

        return output;
    }

    float PID::update(float sig, bool EMA, float alpha){
        float error = sig;
        
        if (start == 0){
          prevError = error;
          start = 1;
        }

        if(EMA){
          error = alpha * error + (1 - alpha) * prevError;
        }
        integral += error;
        derivative = error - prevError;
        output =  kP * error + kI * integral + kD * derivative;
        prevError = error;

        if(sgn(error) != sgn(prevError)) integral = 0;

        if(std::abs(error) < 0.5) integral = 0;
        
        return output;
    }

    float PID::update(float sig, float pos){
        float error = sig - pos;   
        if (start == 0){
          prevError = error;
          start = 1;
        }
        integral += error;
        derivative = error - prevError;
        output =  kP * error + kI * integral + kD * derivative;
        prevError = error;
        return output;
    }

    void PID::reset(){
      integral = 0;
      start = 0;
      prevError = 0;
      out = 0;
      derivative = 0;
      output = 0;
    }

PID angularPID(turn_kp,turn_ki,turn_kd);
PID lateralPID(distance_kp,distance_ki,distance_kd);

chassis* motionChassis;

void initMotion(chassis* initC, std::vector<float> angV, std::vector<float> latV){
   motionChassis = initC;
   angularPID.changeVals(angV[0],angV[1],angV[2]);
   lateralPID.changeVals(latV[0],latV[1],latV[2]);
}

void initMotion(chassis* initC){
   motionChassis = initC;
   angularPID.changeVals(turn_kp,turn_ki,turn_kd);
   lateralPID.changeVals(distance_kp,distance_ki,distance_kd);
}



std::vector<double> toPointStep(float sigX,float sigY, std::vector<double> pos, float angled){
  float wrapped = fmod(((-pos[2])-180),360)+180;
  outA = angularPID.update(fmod(((angled-wrapped)-180),360)+180);
  dist = hypot(sigX-pos[0],sigY-pos[1]);
  outL = lateralPID.update(dist);
  return {outL, outA, wrapped, dist};
  
}

void toPoint(float tarX, float tarY, float exit, moveParams params){
  int count = 0;

  angularPID.reset();
  lateralPID.reset();
  std::vector<double> pos = getPos();
  float dist;
  //float angled = atan2(tarY - pos[1], tarX - pos[0]) * (180/M_PI);
  float targetAngle = atan2(tarX - pos[0], tarY - pos[1]) * (180/M_PI);
  float dx = tarX - pos[0];
  float dy = tarY - pos[1];
  float rawDist;
  int settleCount = 0;
  bool AngSettled = false;
  Brain.Timer.clear();
  if(params.settleLength == 0) params.settleLength = 10;
    do{
      std::vector<double> pos = getPos();
      if(params.reversed) pos[2] += 180;
      
      float dx = tarX - pos[0];
      float dy = tarY - pos[1];
      rawDist = std::hypot(dx, dy); 

      if (!params.steady) targetAngle = atan2(tarX - pos[0], tarY - pos[1]) * (180/M_PI);
 
      float angleError = constrainAngle(targetAngle - pos[2]);

      float angleRad = getAng() * (M_PI / 180.0);
      float forwardX = sin(angleRad);
      float forwardY = cos(angleRad);
      dist = (dx * forwardX + dy * forwardY); 

      if(params.straight || rawDist < params.settleLength) angleError = 0;

      if(!params.cosScalling) dist = dist * std::cos((angleError * M_PI) / 180.0);
      
      if(!params.cosScalling && std::cos((angleError * M_PI) / 180.0) < 0) dist = -dist;
      

      outA = angularPID.update(constrain90(angleError));
      //outA = angularPID.update(angleError);
      outL = lateralPID.update(dist);      

      // if(params.angleExit != 0){
      //   if(std::abs(constrain90(angleError)) < params.angleExit){
      //     outA = 0;
      //     AngSettled = true;
      //   }
      // }else{
      //   if(std::abs(constrain90(angleError)) < 5){
      //     outA = 0;
      //     AngSettled = true;
      //   }
      // }

      // if(std::abs(constrain90(angleError)) < 20){
      //     outA = outA * 0.5;
      // }

      if(AngSettled){
        outA = 0;
      }

      left_out = outL - outA;
      right_out = outL + outA;

      // if(std::abs(angleError) > 120){
      //   left_out = outL;
      //   right_out = outL; 
      // }


      float max = fmax(std::abs(left_out), std::abs(right_out));


      if (params.MAXSPEED == 0) params.MAXSPEED = 100;

      if (max > params.MAXSPEED && params.MAXSPEED > 0){
        left_out = (left_out / max) * params.MAXSPEED;
        right_out = (right_out / max) * params.MAXSPEED;
      }

      count = count + 1;

      motionChassis->leftMotors->spin(vex::forward, left_out, vex::percentUnits::pct);
      motionChassis->rightMotors->spin(vex::forward, right_out, vex::percentUnits::pct);
      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1, 1);
      //Brain.Screen.print("R: %f, L: %d", max, params.cosScalling);
      Brain.Screen.print("X: %f Y: %f Theta %f", getPos()[0], getPos()[1], getPos()[2]);
      Brain.Screen.setCursor(2, 1);
      Brain.Screen.print("Dist: %f Angle: %f", rawDist, angleError);
      Brain.Screen.setCursor(3, 1);
      Brain.Screen.print("L: %f R: %f", left_out, right_out);
      //pros::lcd::print(2, "X: %f Y: %f", pos[0], pos[1]); 
      
      vex::wait(10, vex::msec);

      if (std::abs(rawDist) < exit) {
            settleCount++;
      } else {
            settleCount = 0;
      }

      if (settleCount >= params.settleLength) break;

      if(Brain.Timer.time(vex::msec) > params.timeout) break;
    }while(true);
    left_out = 0;
    right_out = 0;
    // motionChassis->leftMotors->spin(vex::forward, left_out, vex::percentUnits::pct);
    // motionChassis->rightMotors->spin(vex::forward, right_out, vex::percentUnits::pct);
    motionChassis->leftMotors->stop(vex::brakeType::hold);
    motionChassis->rightMotors->stop(vex::brakeType::hold);
}


void toPose(float tarX, float tarY, float tarT, float exit, moveParams params){
  int count = 0;

  angularPID.reset();
  lateralPID.reset();
  std::vector<double> pos = getPos();
  float dist;
  //float angled = atan2(tarY - pos[1], tarX - pos[0]) * (180/M_PI);
  float targetAngle = atan2(tarX - pos[0], tarY - pos[1]) * (180/M_PI);
  float dx = tarX - pos[0];
  float dy = tarY - pos[1];
  float rawDist = hypot(dx, dy);
  float carrDist;
  float angleError;
  Brain.Timer.clear();
    do{
      std::vector<double> pos = getPos();

      Point carr(tarX - rawDist * cos(tarT) * params.dlead,
			         tarY - rawDist * sin(tarT) * params.dlead);
      targetAngle = atan2(carr.x - pos[0], carr.y - pos[1]) * (180/M_PI);

      if(params.reversed) pos[2] += 180;
      
      float Cdx = carr.x - pos[0];
      float Cdy = carr.y - pos[1];
      carrDist = std::hypot(Cdx, Cdy); 
      
      float dx = tarX - pos[0];
      float dy = tarY - pos[1];
      rawDist = std::hypot(dx, dy); 
      if(params.settleDist > 0 && rawDist < params.settleDist){
        targetAngle = tarT;
      }
      angleError = constrainAngle(targetAngle - pos[2]);
    

      float angleRad = getAng() * (M_PI / 180.0);
      float forwardX = sin(angleRad);
      float forwardY = cos(angleRad);
      dist = (Cdx * forwardX + Cdy * forwardY); 

      

      outA = angularPID.update(angleError);

      outL = lateralPID.update(dist);

      left_out = outL - outA;
      right_out = outL + outA;
      count = count + 1;
      motionChassis->leftMotors->spin(vex::forward, left_out, vex::percentUnits::pct);
      motionChassis->rightMotors->spin(vex::forward, right_out, vex::percentUnits::pct);
      //pros::lcd::print(0, "L: %f A: %f", dist, angleError);
      //pros::lcd::print(0, "X: %f Y: %f", carr.x, carr.y); 
      vex::wait(10, vex::msec);

      if(Brain.Timer.time(vex::msec) > params.timeout) break;
    }while(std::abs(rawDist) > exit || std::abs(angleError) > 5);
    left_out = 0;
    right_out = 0;
    motionChassis->leftMotors->spin(vex::forward, left_out, vex::percentUnits::pct);
    motionChassis->rightMotors->spin(vex::forward, right_out, vex::percentUnits::pct);
}


void toAng(float tarT, float exit, moveParams params){

  angularPID.reset();
  lateralPID.reset();
  float err;
  int settleCount = 0;
  Brain.Timer.clear();
    do{
      float angled = constrainAngle(tarT - getAng());
      outA = angularPID.update(constrainAngle(angled));
      left_out = -outA;
      right_out = outA;
      motionChassis->leftMotors->spin(vex::forward, left_out, vex::percentUnits::pct);
      motionChassis->rightMotors->spin(vex::forward, right_out, vex::percentUnits::pct);

      //pros::lcd::print(0, "err: %f", angled);

      vex::wait(10, vex::msec);
      err = angled;
      if (std::abs(err) < exit) {
            settleCount++;
        } else {
            settleCount = 0;
        }

        if (settleCount > 10) break;

        if(Brain.Timer.time(vex::msec) > params.timeout) break;
    }while(true);
    left_out = 0;
    right_out = 0;
    motionChassis->leftMotors->spin(vex::forward, left_out, vex::percentUnits::pct);
    motionChassis->rightMotors->spin(vex::forward, right_out, vex::percentUnits::pct);
    motionChassis->leftMotors->stop();
    motionChassis->rightMotors->stop();
}

void turnToPoint(float tarX, float tarY, float exit, moveParams params){

  angularPID.reset();
  lateralPID.reset();
  float err;
  std::vector<double> pos = getPos();
  float targetAngle = atan2(tarX - pos[0], tarY - pos[1]) * (180/M_PI);
    int settleCount = 0;

    Brain.Timer.clear();
    do{

      float currentHeading = getAng();
      float angleError = targetAngle - currentHeading;

      angleError = constrainAngle(angleError);

      outA = angularPID.update(angleError);
      left_out = -outA;
      right_out = outA;
      motionChassis->leftMotors->spin(vex::forward, left_out, vex::percentUnits::pct);
      motionChassis->rightMotors->spin(vex::forward, right_out, vex::percentUnits::pct);

      //pros::lcd::print(0, "X: %f Y: %f error: %f", pos[0], pos[1], angleError);

      vex::wait(10, vex::msec);
      err = angleError;
      if (std::abs(err) < exit) {
            settleCount++;
        } else {
            settleCount = 0;
        }

        if (settleCount > 10) break;

        if(Brain.Timer.time(vex::msec) > params.timeout) break;
    }while(true);
    //}while(1==2);
    left_out = 0;
    right_out = 0;
    motionChassis->leftMotors->spin(vex::forward, left_out, vex::percentUnits::pct);
    motionChassis->rightMotors->spin(vex::forward, right_out, vex::percentUnits::pct);
}

void arcToDist(float tarT, float exit, float leftSpeed, float rightSpeed, moveParams params){
int count = 0;

  angularPID.reset();
  lateralPID.reset();
  std::vector<double> startPos = getPos();
  float rawDist;
  float remainDist;
  int settleCount = 0;
  float err;
  Brain.Timer.clear();

  if(params.settleLength == 0) params.settleLength = 10;
  do{
      float angled = constrainAngle(tarT - getAng());

      outL = lateralPID.update(remainDist);      

      left_out = outL;
      right_out = outL;

      if (std::abs(leftSpeed) > std::abs(rightSpeed)){
        if (std::abs(left_out) > leftSpeed) left_out = leftSpeed * sgn(left_out);
        right_out = (leftSpeed * right_out) / rightSpeed;
      }

      if (std::abs(leftSpeed) < std::abs(rightSpeed)){
        if (std::abs(right_out) > rightSpeed) right_out = rightSpeed * sgn(right_out);
        left_out = (rightSpeed * left_out) / leftSpeed;
      }
      
      count = count + 1;

      motionChassis->leftMotors->spin(vex::forward, leftSpeed, vex::percentUnits::pct);
      motionChassis->rightMotors->spin(vex::forward, rightSpeed, vex::percentUnits::pct);
      //pros::lcd::print(0, "L: %f A: %f", rawDist, remainDist);
      //pros::lcd::print(2, "X: %f Y: %f", pos[0], pos[1]);

      vex::wait(10, vex::msec);
      err = angled;
      if (std::abs(err) < exit) {
            settleCount++;
        } else {
            settleCount = 0;
        }

        if (settleCount >= params.settleLength) break;

        if(Brain.Timer.time(vex::msec) > params.timeout) break;
    }while(true);
    left_out = 0;
    right_out = 0;
    motionChassis->leftMotors->spin(vex::forward, left_out, vex::percentUnits::pct);
    motionChassis->rightMotors->spin(vex::forward, right_out, vex::percentUnits::pct);
}


void toDist(float tarDist, int dir, float exit, moveParams params){
  int count = 0;

  angularPID.reset();
  lateralPID.reset();
  std::vector<double> pos = getPos();
  std::vector<double> startPos = pos;
  float dist;
  float prevOutL = 0;
  //float angled = atan2(tarY - pos[1], tarX - pos[0]) * (180/M_PI);
  //float targetAngle = atan2(tarX - pos[0], tarY - pos[1]) * (180/M_PI);
  // float dx = tarX - pos[0];
  // float dy = tarY - pos[1];
  float rawDist;
  int settleCount = 0;
  Brain.Timer.clear();
  if(params.settleLength == 0) params.settleLength = 10;
    do{
      std::vector<double> pos = getPos();
      if(params.reversed) pos[2] += 180;
      

      rawDist = tarDist - (std::hypot(pos[0] - startPos[0], pos[1] - startPos[1])); 


      // float angleRad = getAng() * (M_PI / 180.0);
      // float forwardX = sin(angleRad);
      // float forwardY = cos(angleRad);
      // dist = (dx * forwardX + dy * forwardY); 

      // if(params.straight || rawDist < params.settleDist) angleError = 0;

      // if(!params.cosScalling) dist = dist * std::cos((angleError * M_PI) / 180.0);
      
      // if(!params.cosScalling && std::cos((angleError * M_PI) / 180.0) < 0) dist = 0;
      

      outA = 0;
      outL = lateralPID.update(rawDist);
      outL *= sgn(dir);

      if(params.latAccelSlew != 0.0 ){
        if(outL > prevOutL + params.latAccelSlew) outL = prevOutL + params.latAccelSlew;
      }

      if(params.latDecelSlew != 0.0 ){
        if(outL < prevOutL - params.latDecelSlew) outL = prevOutL - params.latDecelSlew;
      }


      prevOutL = outL;
      left_out = outL - outA;
      right_out = outL + outA;


      float max = fmax(std::abs(left_out), std::abs(right_out));
      //pros::lcd::print(0, "R: %f, L: %f", max, params.MAXSPEED);

      if (params.MAXSPEED == 0) params.MAXSPEED = 100;

      if (max > params.MAXSPEED && params.MAXSPEED > 0){
        left_out = (left_out / max) * params.MAXSPEED;
        right_out = (right_out / max) * params.MAXSPEED;
      }

      count = count + 1;

      motionChassis->leftMotors->spin(vex::forward, left_out, vex::percentUnits::pct);
      motionChassis->rightMotors->spin(vex::forward, right_out, vex::percentUnits::pct);
      //pros::lcd::print(0, "L: %f A: %f", rawDist, angleError);
      //pros::lcd::print(2, "X: %f Y: %f", pos[0], pos[1]); 
      
      vex::wait(10, vex::msec);

      if (std::abs(rawDist) < exit) {
            settleCount++;
      } else {
            settleCount = 0;
      }

      if (settleCount >= params.settleLength) break;

      if(Brain.Timer.time(vex::msec) > params.timeout) break;
    }while(true);
    left_out = 0;
    right_out = 0;
    motionChassis->leftMotors->spin(vex::forward, left_out, vex::percentUnits::pct);
    motionChassis->rightMotors->spin(vex::forward, right_out, vex::percentUnits::pct);
}


float pt_to_pt_distance (Point pt1,Point pt2){
    float distance = std::sqrt(std::pow((pt2.x - pt1.x), 2) + std::pow((pt2.y - pt1.y),2));
    return distance;
}

float pt_to_pt_distance (Point pt1,std::pair<float, float> pt2){
    float distance = std::sqrt(std::pow((pt2.first - pt1.x), 2) + std::pow((pt2.second - pt1.y),2));
    return distance;
}