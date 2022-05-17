#include <webots/Supervisor.hpp>

#define TIME_STEP 32

using namespace webots;

int main() {
  Supervisor *supervisor = new Supervisor();

  // do this once only
  Node *robot_node = supervisor->getFromDef("red");
  Node *robot_node1 = supervisor->getFromDef("green");
  Node *robot_node2 = supervisor->getFromDef("blue");
  if (robot_node == NULL) {
    std::cerr << "No DEF MY_ROBOT node found in the current world file" << std::endl;
    exit(1);
  }
  Field *trans_field = robot_node->getField("translation");
  Field *trans_field1 = robot_node1->getField("translation");
  Field *trans_field2 = robot_node2->getField("translation");

  while (supervisor->step(TIME_STEP) != -1) {
    // this is done repeatedly
    const double *values = trans_field->getSFVec3f();
    std::cout << "MY_ROBOT is at position: " << values[0] << ' '
              << values[1] << ' ' << values[2] << std::endl;
    const double *values1 = trans_field1->getSFVec3f();
    std::cout << "MY_ROBOT1 is at position: " << values1[0] << ' '
              << values1[1] << ' ' << values1[2] << std::endl;
    const double *values2 = trans_field2->getSFVec3f();
    std::cout << "MY_ROBOT2 is at position: " << values2[0] << ' '
              << values2[1] << ' ' << values2[2] << std::endl;
          
  }

  delete supervisor;
  return 0;
}