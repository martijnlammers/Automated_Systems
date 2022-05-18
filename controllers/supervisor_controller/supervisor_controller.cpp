#include <webots/Supervisor.hpp>
#include <webots/Motor.hpp>
#include <string>
#include <iostream>
#include <cstdlib>

#define TIME_STEP 64
#define MAX_SPEED 10.28
#define LEFTMOTOR 0
#define RIGHTMOTOR 1

using namespace webots;
using namespace std;

Supervisor *supervisor = new Supervisor();
Node *robot_node;

void rotateRobot(const string &name, int degrees){
  robot_node = supervisor->getFromDef(name);
  Field *r_field = robot_node->getField("rotation");
  
  //Get the rotation values from the node field.
  const double *r_val = r_field->getSFRotation();
  const double new_r_val[4] = {r_val[0], r_val[1], r_val[2], (r_val[3] + (degrees * 0.0174532925))};
  r_field->setSFRotation(new_r_val);
}

void moveToLocation(const string &name, double x, double y){
  robot_node = supervisor->getFromDef(name);
  if (robot_node == NULL) exit(1);
   
  Field *trans_field = robot_node->getField("translation");
  const double *values = trans_field->getSFVec3f();
  cout << "MY_ROBOT is at position: " << values[0] << ' ' << values[1] << ' ' << values[2] << endl;  
  const double newValue[3] = {x, y, values[2]};
  trans_field->setSFVec3f(newValue);
}
  
  
int main() {
  double x = -0.52;
  double y = -0.44;
  // while (supervisor->step(TIME_STEP) != -1) {
        // moveBot("red", x, y); 
  // }
  rotateRobot("red", 180);
  //moveToLocation("red", x, y); 
  delete supervisor;
  return 0;
}