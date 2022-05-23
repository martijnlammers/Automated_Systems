// Automated Systems Webots simulation controller.
// Martijn Lammers
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/GPS.hpp>
#include <iostream>
#include <math.h>
#include <vector>

#define TIME_STEP 64
#define ROTATE_SPEED 0.07
#define MAX_SPEED 18
#define LEFT 0
#define RIGHT 1
#define NUM_MOTORS 2


// Please don't touch these.
#define POS_MATCHING_ACC 0.025
#define ANGLE_ACCURACY 0.001

using namespace webots;
using namespace std;

Robot *r = new Robot();
Motor *motors[2] = {r->getMotor("wheel_left"), r->getMotor("wheel_right")};
GPS *m_gps = r->getGPS("middleGPS");
GPS *f_gps = r->getGPS("frontGPS");
vector<double> destinationCoordinates;

// Basic movement for the robot.
void motorStop(){
    motors[LEFT]->setVelocity(0);
    motors[RIGHT]->setVelocity(0);
    r->step(1);
}

void motorMoveForward(double speed_multiplier){
    motors[LEFT]->setVelocity(speed_multiplier * MAX_SPEED);
    motors[RIGHT]->setVelocity(speed_multiplier * MAX_SPEED);
    r->step(1);
}

void motorRotateLeft(double speed_multiplier){
    motors[LEFT]->setVelocity(speed_multiplier * -ROTATE_SPEED);
    motors[RIGHT]->setVelocity(speed_multiplier * ROTATE_SPEED);
    r->step(1);
}

void motorRotateRight(double speed_multiplier){
    motors[LEFT]->setVelocity(speed_multiplier * ROTATE_SPEED);
    motors[RIGHT]->setVelocity(speed_multiplier * -ROTATE_SPEED);
    r->step(1);
}

// Set up of sensors and actuators used by the robot.
void initializeRobot(){
  m_gps->enable(TIME_STEP);
  f_gps->enable(TIME_STEP);
  motors[LEFT]->setPosition(INFINITY);
  motors[RIGHT]->setPosition(INFINITY);
  motorStop();
  string name = r->getName();

  //Test scenario with 3 bots.
  if(name == "robot(1)"){
    destinationCoordinates.push_back(-0.4);
    destinationCoordinates.push_back(-0.13);
  } else if(name == "robot"){
    destinationCoordinates.push_back(0.15);
    destinationCoordinates.push_back(-0.6);
  } else if(name == "robot(2)"){
    destinationCoordinates.push_back(-0.4);
    destinationCoordinates.push_back(-1.07);
  } else if(name == "robot(3)"){
    destinationCoordinates.push_back(-0.4);
    destinationCoordinates.push_back(-0.6);
  }
}







/* Math source: https://www.cuemath.com/geometry/angle-between-vectors/
 * Calculates the rotation to the target.
 */ 
 
double calculateAngleToTarget(vector<vector<double>> vectors){
  double a = vectors[0][0] ,
  b = vectors[0][1],
  c = vectors[1][0],
  d = vectors[1][1];
  double dotProduct = ((a * c) + (b * d));
  return acos(dotProduct / 
  (sqrt(pow(a, 2) + pow(b,2)) * 
  sqrt(pow(c, 2) + pow(d,2))))
  * 57.2957795;
}

vector<vector<double>> getVectors(vector<double> &destination){
  const double *m_robotPos = m_gps->getValues();
  const double *f_robotPos = f_gps->getValues();
  
  /* Calculates two vectors;
   * Vector 1: Target(x,y) to MiddleRobot(x,y)
   * Vector 2: MiddleRobot(x,y) to FrontRobot(x,y)
   */
   
  vector<vector<double>> vectors{{(destination[0] - m_robotPos[0]), (destination[1] - m_robotPos[1])},
  {(f_robotPos[0] - m_robotPos[0]), (f_robotPos[1] - m_robotPos[1])}};
  return vectors;
}


void rotateToTarget(vector<double> &destination){
    double initialAngle = calculateAngleToTarget(getVectors(destination));
    double newAngle = calculateAngleToTarget(getVectors(destination));
    
    /* This block is used to determine to rotate 
     * clockwise or counter clockwise.
     */
     
    while(initialAngle == calculateAngleToTarget(getVectors(destination))){
      motorRotateLeft(newAngle);
    }
    motorStop();
    if(initialAngle > newAngle){
      while(newAngle > ANGLE_ACCURACY){
        motorRotateLeft(newAngle);
        newAngle = calculateAngleToTarget(getVectors(destination));
      }
    } else {
      while(newAngle > ANGLE_ACCURACY){
        motorRotateRight(newAngle);
        newAngle = calculateAngleToTarget(getVectors(destination));
      }
    }
    motorStop();
    return;
    
}

void moveToTarget(vector<double> &destination){
  double a,b,length;
  
  do {
    vector<vector<double>> vectors = getVectors(destination); 
    a = vectors[0][0];
    b = vectors[0][1];
    length = sqrt(pow(a, 2) + pow(b,2));
    motorMoveForward(length);
  } while (length > POS_MATCHING_ACC);
  motorStop();
  return;
}

void moveRobot(vector<double> &destination){
     r->step(1);
     vector<vector<double>> vectors = getVectors(destination);
     // Quits function when the destination equals the current position.
     // if (fabs(m_robotPos[0] - destination[0]) < POS_MATCHING_ACC &&
      // fabs(m_robotPos[1] - destination[1]) < POS_MATCHING_ACC) return;
     //cout << calculateAngleToTarget(getVectors(destination)) << endl;
     rotateToTarget(destination); 
     moveToTarget(destination); 
}

int main(int argc, char **argv) {
   r->step(1);
   initializeRobot();
   moveRobot(destinationCoordinates);
   
   // Just for fun.
   string name = r->getName();
   if(name == "robot(1)"){
    destinationCoordinates[0] = -0.58;
    destinationCoordinates[1] = -0.13;
  } else if(name == "robot"){
    destinationCoordinates[0] = 0.19;
    destinationCoordinates[1] = -0.13;
  } else if(name == "robot(2)"){
    destinationCoordinates[0] = 0.19;
    destinationCoordinates[1] = -1.07;
  } else if(name == "robot(3)"){
    destinationCoordinates[0] = -0.58;
    destinationCoordinates[1] = -1.07;
  }
  moveRobot(destinationCoordinates);
  
  
   delete r;
   return 0;
}
