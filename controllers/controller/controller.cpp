#include <webots/Robot.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Motor.hpp>
#include <webots/Compass.hpp>
#include <webots/GPS.hpp>
#include <string>
#include <iostream>

// time in [ms] of a simulation step
#define TIME_STEP 64
#define MAX_SPEED 10.28
#define LEFT 0
#define RIGHT 1
#define NUM_MOTORS 2
#define NEGATIVE_DEG degrees < 0
#define POSITIVE_DEG degrees > 0

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

Robot *r = new Robot();
Motor *motors[2] = {r->getMotor("wheel_left"), r->getMotor("wheel_right")};
GPS *gps = r->getGPS("global");
//Accelerometer *accMeter = r->getAccelerometer("accelerometer");
Compass *compass = r->getCompass("compass");


// Motor *rightMotor = robot->getMotor("wheel_right");
// enum string_code{eForward,
  // eBackward, eRotate, eInvalid
// };
void initializeRobot(){
  gps->enable(TIME_STEP);
  //accMeter->enable(TIME_STEP);
  compass->enable(TIME_STEP);
  
}
void rotateToTarget(float x, float y){
  // Current position
  // Make vector
  // calculate orientation
  // 
}

void moveRobot(float x, float y){
     motors[LEFT]->setPosition(INFINITY);
     motors[RIGHT]->setPosition(INFINITY);
     motors[LEFT]->setVelocity(0.1 * MAX_SPEED);
     motors[RIGHT]->setVelocity(-0.1 * MAX_SPEED);
     cout << "coordinates: " << gps->getValues()[0] << endl;
     cout << "coordinates: " << gps->getValues()[1] << endl;
     cout << "coordinates: " << gps->getValues()[2] << endl;
     // cout << "accelerometer: " << accMeter->getValues()[0] << endl;
     // cout << "accelerometer: " << accMeter->getValues()[1] << endl;
     // cout << "accelerometer: " << accMeter->getValues()[2] << endl;
     cout << "compass: " << compass->getValues() << endl;
  
     rotateToTarget(x, y);
     
}

int main(int argc, char **argv) {
   initializeRobot();
   float x = -0.06, y = -0.06;
   while(r->step(TIME_STEP) != -1){
     moveRobot(x, y);
   }
   
   delete r;
   return 0; //EXIT_SUCCESS
}
