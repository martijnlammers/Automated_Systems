// Automated Systems Webots simulation controller.
// Martijn Lammers
#include <webots/Robot.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Motor.hpp>
#include <webots/Compass.hpp>
#include <webots/GPS.hpp>
#include <iostream>
#include <math.h>

// time in [ms] of a simulation step
#define TIME_STEP 64
#define MAX_SPEED 10.28
#define LEFT 0
#define RIGHT 1
#define NUM_MOTORS 2
#define NEGATIVE_DEG degrees < 0
#define POSITIVE_DEG degrees > 0
#define POS_MATCHING_ACC 0.01

#define THETA_MATCHING_ACCURACY 1

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

Robot *r = new Robot();
Motor *motors[2] = {r->getMotor("wheel_left"), r->getMotor("wheel_right")};
GPS *gps = r->getGPS("global");
// Accelerometer *accMeter = r->getAccelerometer("accelerometer");
Compass *compass = r->getCompass("compass");


// Motor *rightMotor = robot->getMotor("wheel_right");
// enum string_code{eForward,
  // eBackward, eRotate, eInvalid
// };
void enableSensors(){
  gps->enable(TIME_STEP);
  compass->enable(TIME_STEP);
}
void rotateToTarget(double *destinationCoordinates){
  // Current position
  // Make vector
  // calculate orientation
  // 
}


double cartesianConvertCompassBearingToHeading(double heading) {
    heading = 360-heading;
    heading = heading + 90;
    if (heading > 360.0)
        heading = heading - 360.0;

    return heading;
}

double cartesianCalcThetaDot(double heading, double destinationTheta) {
    double theta_dot = destinationTheta - heading;

    if (theta_dot > 180)
        theta_dot = -(360-theta_dot);
    else if (theta_dot < -180)
        theta_dot = (360+theta_dot);

    return theta_dot;
}


// Code inspiration: https://cyberbotics.com/doc/reference/compass
double getRobotBearing()
{
    /* calculate bearing angle in degrees */
    const double *north = compass->getValues();
    double rad = atan2(north[0], north[2]);
    double bearing = (rad - 1.5708) / M_PI * 180.0;
    if (bearing < 0.0) {
        bearing = bearing + 360.0;
	}
    return bearing;
}

void moveRobot(double *destinationCoordinates){
     const double * currentCoordinates = gps->getValues();
     
     // Code inspiration: https://github.com/albertbrucelee/webots-e-puck_robot-tutorial/blob/master/3%20-%20Move%20To%20Destination%20Location/source%20code/controllers/e-puck-move_to_destination_location/e-puck-move_to_destination_location.c
     
     // Returns moving the robot if it's
     // already on thesame coordinates.
     if (fabs(currentCoordinates[0] - destinationCoordinates[0]) < POS_MATCHING_ACC &&
      fabs(currentCoordinates[1] - destinationCoordinates[1]) < POS_MATCHING_ACC) return;
  
  
     // Get the angle to rotate to the target position	
     double heading = cartesianConvertCompassBearingToHeading(getRobotBearing());
     
     
     double destinationTheta = atan2(destinationCoordinate[1] - currentCoordinate[1], destinationCoordinate[0] - currentCoordinate[0]) * 180 / M_PI;
     double thetaDotToDestination = cartesianCalcThetaDot(heading, destinationTheta);
     
     
     motors[LEFT]->setPosition(INFINITY);
     motors[RIGHT]->setPosition(INFINITY);
     motors[LEFT]->setVelocity(0.1 * MAX_SPEED);
     motors[RIGHT]->setVelocity(-0.1 * MAX_SPEED);
     cout << "coordinates: " << currentCoordinates[0] << endl;
     cout << "coordinates: " << currentCoordinates[1] << endl;
     cout << "coordinates: " << currentCoordinates[2] << endl;
     
     cout << "\ncompass X: " << compass->getValues()[0] << endl;
     cout << "compass Y: " << compass->getValues()[1] << endl;
     cout << "compass Z: " << compass->getValues()[2] << endl;
     rotateToTarget(destinationCoordinates);
     
}
void communicateToServer(){
  // To-do 
  // Subscribe to MQTT server to write and fetch data from.
}

int main(int argc, char **argv) {
   enableSensors();
   // double destinationCoordinates[2] = {-0.000000, -0.0094};
   double destinationCoordinates[2] = {-0.06, -0.06};
   while(r->step(TIME_STEP) != -1){
     moveRobot(destinationCoordinates);
     cout << "hello" << endl;
   }
   
   delete r;
   return 0; //EXIT_SUCCESS
}
