#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <string>
#include <iostream>

// time in [ms] of a simulation step
#define TIME_STEP 64
#define MAX_SPEED 10.28
#define LEFTMOTOR 0
#define RIGHTMOTOR 1
#define NUM_MOTORS 2
#define NEGATIVE_DEG degrees < 0
#define POSITIVE_DEG degrees > 0

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;


enum string_code{
  eForward,
  eBackward,
  eStop,
  eRotate,
  eInvalid
};

// In a function as its used in an overload.
string_code encodeDirection(string direction){
  string_code code = (direction == "FORWARD") ? eForward :
  (direction == "BACKWARD") ? eBackward : 
  (direction == "ROTATE") ? eRotate : eInvalid;
  return code;
}

void changeDirection(Robot *robot, string direction){
  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("right wheel motor");
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  switch(encodeDirection(direction)){
  case eForward:
     leftMotor->setVelocity(0.1 * MAX_SPEED);
     rightMotor->setVelocity(0.1 * MAX_SPEED);
    
    cout << "FORWARD"<< endl;
    break;
  case eBackward:
     leftMotor->setVelocity(-(0.1 * MAX_SPEED));
     rightMotor->setVelocity(-(0.1 * MAX_SPEED));
    cout << "BACKWARD"<< endl;
    break;
  case eStop:
     leftMotor->setVelocity(0);
     rightMotor->setVelocity(0);
  default:
    //cout << "Invalid movement. Only 'FORWARD', 'BACKWARD' or 'ROTATE'" << end;
    cout << "Invalid use of function." << endl;
    break;
  }
  
}

void changeDirection(Robot *robot, string direction, int degrees){
  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("right wheel motor");
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  switch(encodeDirection(direction)){
  case eRotate:
    cout << "ROTATE, Rotation in deg: " << degrees << endl;
    if(POSITIVE_DEG){
      leftMotor->setVelocity(0.1 * MAX_SPEED);
      rightMotor->setVelocity(-0.1 * MAX_SPEED);
    } else if(NEGATIVE_DEG){
      cout << "ROTATE, Rotation in deg: " << degrees << endl;
      leftMotor->setVelocity(-0.1 * MAX_SPEED);
      rightMotor->setVelocity(0.1 * MAX_SPEED);
    }
    
    break;
  default:
    cout << "Invalid use of function." << endl;
    break;
  }
}


// entry point of the controller
int main(int argc, char **argv) {
  // create the Robot instance.
  
  // leftMotor->setPosition(INFINITY);
  // rightMotor->setPosition(INFINITY);
  // leftMotor->setVelocity(0.0);
  // rightMotor->setVelocity(0.0);
   Robot *robot = new Robot();
   //while (robot->step(TIME_STEP) != -1);
   changeDirection(robot, "ROTATE", 90);
  
   //changeDirection(robot, "BACKWARD");
  
   delete robot;
   return 0; //EXIT_SUCCESS
}
