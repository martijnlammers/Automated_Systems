#include <webots/Supervisor.hpp>

#define TIME_STEP 32

using namespace webots;
using namespace std;

int main() {
  Supervisor *supervisor = new Supervisor();

  // do this once only
  // Node *robot_node = supervisor->getFromDef("MY_ROBOT");
  // if (robot_node == NULL) {
    // std::cerr << "No DEF MY_ROBOT node found in the current world file" << std::endl;
    // exit(1);
  // }
  // Field *trans_field = robot_node->getField("translation");

  // while (supervisor->step(TIME_STEP) != -1) {
    //this is done repeatedly
    // const double *values = trans_field->getSFVec3f();
    // std::cout << "MY_ROBOT is at position: " << values[0] << ' '
              // << values[1] << ' ' << values[2] << std::endl;
  // }
  //cout << "hello" << end;

  delete supervisor;
  return 0;
}