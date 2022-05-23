// Automated Systems Webots simulation controller.
// Martijn Lammers
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/GPS.hpp>
#include <iostream>
#include <math.h>
#include <vector>

#define TIME_STEP 64
#define ROTATE_SPEED 0.9
#define MAX_SPEED 6.28
#define LEFT 0
#define RIGHT 1
#define NUM_MOTORS 2
#define POS_MATCHING_ACC 0.01
#define ANGLE_ACCURACY 1

using namespace webots;
using namespace std;

Robot *r = new Robot();
Motor *motors[2] = {r->getMotor("wheel_left"), r->getMotor("wheel_right")};
GPS *m_gps = r->getGPS("middleGPS");
GPS *f_gps = r->getGPS("frontGPS");


//Set up of sensors and actuators used by the robot.
void initializeRobot(){
  m_gps->enable(TIME_STEP);
  f_gps->enable(TIME_STEP);
  motors[LEFT]->setPosition(INFINITY);
  motors[RIGHT]->setPosition(INFINITY);
  motors[LEFT]->setVelocity(0);
  motors[RIGHT]->setVelocity(0);
}

//Basic movement for the robot
void motorStop(){
    motors[LEFT]->setVelocity(0);
    motors[RIGHT]->setVelocity(0);
}

void motorMoveForward(){
    motors[LEFT]->setVelocity(MAX_SPEED);
    motors[RIGHT]->setVelocity(MAX_SPEED);
}

void motorRotateLeft(){
    motors[LEFT]->setVelocity(-ROTATE_SPEED);
    motors[RIGHT]->setVelocity(ROTATE_SPEED);
}

void motorRotateRight(){
    motors[LEFT]->setVelocity(ROTATE_SPEED);
    motors[RIGHT]->setVelocity(-ROTATE_SPEED);
}



// Math source: https://www.cuemath.com/geometry/angle-between-vectors/
// Calculates the rotation to the target.
double calculateAngleToTarget(vector<vector<double>> vectors){
  double a = vectors[0][0] ,
  b = vectors[0][1],
  c = vectors[1][0],
  d = vectors[1][1];
  cout << a << " " <<  b << endl;
  cout << c << " " << d << endl;
  double dotProduct = ((a * c) + (b * d));
  cout << dotProduct << endl;
  return acos(dotProduct / 
  (sqrt(pow(a, 2) + pow(b,2)) * 
  sqrt(pow(c, 2) + pow(d,2))))
  * 57.2957795;
}

vector<vector<double>> getVectors(vector<double> &destination){
  const double *m_robotPos = m_gps->getValues();
  const double *f_robotPos = f_gps->getValues();
  vector<vector<double>> vectors{{(destination[0] - m_robotPos[0]), (destination[1] - m_robotPos[1])},
  {(f_robotPos[0] - m_robotPos[0]), (f_robotPos[1] - m_robotPos[1])}};
  return vectors;
}


void rotateToTarget(vector<double> &destination){
    double initialAngle = calculateAngleToTarget(getVectors(destination));
    while(initialAngle == calculateAngleToTarget(getVectors(destination))){
      motorRotateLeft();
      r->step(1);
    }
    motorStop();
    r->step(1);
    double newAngle = calculateAngleToTarget(getVectors(destination));
    cout << initialAngle << " " << newAngle << endl;
 
    if(initialAngle > newAngle){
      while( calculateAngleToTarget(getVectors(destination)) > ANGLE_ACCURACY){
        motorRotateLeft();
        r->step(1);
        cout << "hello " << endl;
        cout << calculateAngleToTarget(getVectors(destination)) << endl;
        cout << ANGLE_ACCURACY << endl;
      }
    } else {
      while(calculateAngleToTarget(getVectors(destination)) > ANGLE_ACCURACY){
        motorRotateRight();
        r->step(1);
        cout << "hello2 " << endl;
        cout << calculateAngleToTarget(getVectors(destination)) << endl;
        cout << ANGLE_ACCURACY << endl;
      }
    }
    motorStop();
    r->step(1);
    return;
    
}

void moveForward(double distance)
{
	
}

void moveRobot(vector<double> &destination){
     r->step(10);
     vector<vector<double>> vectors = getVectors(destination);
     // Quits function when the destination equals the current position.
     // if (fabs(m_robotPos[0] - destination[0]) < POS_MATCHING_ACC &&
      // fabs(m_robotPos[1] - destination[1]) < POS_MATCHING_ACC) return;
     //cout << calculateAngleToTarget(getVectors(destination)) << endl;
     rotateToTarget(destination); 
}

int main(int argc, char **argv) {
   initializeRobot();
   vector<double> destinationCoordinates{-0.258, -0.45};
   r->step(1);
   moveRobot(destinationCoordinates);
   delete r;
   return 0; //EXIT_SUCCESS
}
