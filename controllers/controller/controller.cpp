// Automated Systems Webots simulation controller.
// Martijn Lammers


/* Notable bugs:
*  - If the robot doesn't reach the destination withing the positional accuracy,
*  it will get angry and decide to run from the problem, just like an upset adolecent. 
*  If this happens, the constant POS_MATCHING_ACC needs to be higher. 
*  (Increase in increments of 0.005)
*  
*
*
*/

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/GPS.hpp>
#include <webots/LightSensor.hpp>
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
#define POS_MATCHING_ACC 0.005
#define ANGLE_ACCURACY 0.001

using namespace webots;
using namespace std;


Robot *r = new Robot();
Motor *motors[2] = {r->getMotor("wheel_left"), r->getMotor("wheel_right")};
GPS *m_gps = r->getGPS("middleGPS");
GPS *f_gps = r->getGPS("frontGPS");
LightSensor *lightsensor = r->getLightSensor("light sensor");
bool found_target = false;
// vector<double> destinationCoordinates{-1.178e-05, -0.00948605};
vector<double> destinationCoordinates{-0.4, -0.13};

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
  lightsensor->enable(TIME_STEP);
  motors[LEFT]->setPosition(INFINITY);
  motors[RIGHT]->setPosition(INFINITY);
  motorStop();
  string name = r->getName();
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
    
    /* Rotate wheels until the angle is close 
     * to 0 deg.
     */
     
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
     const double *m_robotPos = m_gps->getValues();
     vector<vector<double>> vectors = getVectors(destination);
     
     // Quits function when the destination equals the current position.
     if (fabs(m_robotPos[0] - destination[0]) < POS_MATCHING_ACC &&
      fabs(m_robotPos[1] - destination[1]) < POS_MATCHING_ACC) return;
      
     rotateToTarget(destination); 
     moveToTarget(destination); 
}

int main(int argc, char **argv) {
   r->step(1);
   initializeRobot();
   moveRobot(destinationCoordinates);
   delete r;
   return 0;
   /* TO-DO
   * Create event handler for sensors
   * Connect to server using MQTT
   * Add LDR + IR(?) sensors
   */
   
}
