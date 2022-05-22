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
#define MAX_SPEED 6.28
#define LEFT 0
#define RIGHT 1
#define NUM_MOTORS 2
#define NEGATIVE_DEG degrees < 0
#define POSITIVE_DEG degrees > 0
#define POS_MATCHING_ACC 0.01
#define THETA_MATCHING_ACCURACY 1

// tangensial/linear speed in m/s. 
// Tangensial speed = angular speed * wheel radius 
// Tangensial speed = 6.28 rad * 0.03 m = 0.1884 m/s
#define TANGENSIAL_SPEED 0.1884

// Speed of robot to spinning in place (in cycles per second)
// 1 cycle = 360 degrees. 
// Robot rotational speed = tangensial speed / (phi * axle length) 
// note: axle length is distance between wheels
// Robot rotational speed = 0.1884 / (phi*0.08) = 0.74961978196
#define ROBOT_ROTATIONAL_SPEED 0.74961978196

// Speed of robot to spinning in place (in degrees per second)
// Robot angular speed in degrees = robot rotational speed * 360 
// Robot angular speed in degrees = 0.74961978196*360 = 283.588111888 
#define ROBOT_ANGULAR_SPEED_IN_DEGREES 269.863121506



// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

Robot *r = new Robot();
Motor *motors[2] = {r->getMotor("wheel_left"), r->getMotor("wheel_right")};
GPS *gps = r->getGPS("global");
Compass *compass = r->getCompass("compass");

//Set up of sensors and actuators used by the robot.
void initializeRobot(){
  gps->enable(TIME_STEP);
  compass->enable(TIME_STEP);
  motors[LEFT]->setPosition(INFINITY);
  motors[RIGHT]->setPosition(INFINITY);
  motors[LEFT]->setVelocity(0);
  motors[RIGHT]->setVelocity(0);
}

void motorStop(){
    motors[LEFT]->setVelocity(0);
    motors[RIGHT]->setVelocity(0);
}

void motorMoveForward(){
    motors[LEFT]->setVelocity(MAX_SPEED);
    motors[RIGHT]->setVelocity(MAX_SPEED);
}

void motorRotateLeft(){
    motors[LEFT]->setVelocity(-MAX_SPEED);
    motors[RIGHT]->setVelocity(MAX_SPEED);
}

void motorRotateRight(){
    motors[LEFT]->setVelocity(MAX_SPEED);
    motors[RIGHT]->setVelocity(-MAX_SPEED);
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
bool cartesianIsThetaEqual(const double theta, const double theta2){
    if (fabs(theta - theta2) < THETA_MATCHING_ACCURACY) return true;
    return false;
}


void rotateHeading(const double thetaDot){
	if (!cartesianIsThetaEqual(thetaDot, 0)){
		// the duration required for the robot to rotate the body by the specified thetaDot
		double duration = abs(thetaDot) / ROBOT_ANGULAR_SPEED_IN_DEGREES;
		printf("duration to face the destination: %.5f\n", duration);

		if (thetaDot > 0){
                    	   motorRotateRight();
		} else if (thetaDot < 0){
		   motorRotateLeft();
		}
		double start_time = r->getTime();
		do
		{
			r->step(1);
		}
		while (r->getTime() < start_time + duration);
	}
}

void moveForward(double distance)
{
	// the duration required for the robot to move by the specified distance
	double duration = distance / TANGENSIAL_SPEED;
	printf("duration to reach target location: %.5f\n", duration);

	// set robot motor to move forward
	motorMoveForward();

	// run the simulator
	double start_time = r->getTime();
		do
		{
			r->step(1);
		}
		while (r->getTime() < start_time + duration);
	
	// stop the motor
        motorStop();
	r->step(1);
}

void moveRobot(double *destination){
     r->step(1);
     const double * robotPosition = gps->getValues();
     // Code inspiration: https://github.com/albertbrucelee/webots-e-puck_robot-tutorial/blob/master/3%20-%20Move%20To%20Destination%20Location/source%20code/controllers/e-puck-move_to_destination_location/e-puck-move_to_destination_location.c
     
     printf("Initial Coordinate: %.5f %.5f\n", robotPosition[0], robotPosition[1]);
     printf("Destination Coordinate: %.5f %.5f\n", destination[0], destination[1]);
     
     // Quits function when the destination equals the current position.
     if (fabs(robotPosition[0] - destination[0]) < POS_MATCHING_ACC &&
      fabs(robotPosition[1] - destination[1]) < POS_MATCHING_ACC) return;
  
     // Get the angle to destination and rotate.	
     double robotHeading = cartesianConvertCompassBearingToHeading(getRobotBearing());
     double destinationTheta = atan2(destination[1] - robotPosition[1], destination[0] - robotPosition[0]) * 180 / M_PI;
     double thetaDotToDestination = cartesianCalcThetaDot(robotHeading, destinationTheta);
     printf("thetaDotToDestination: %.5f\n", thetaDotToDestination);
     rotateHeading(thetaDotToDestination);

     // the distance needed for the robot to reach its destination
     double distanceToDestination = sqrt(pow(destination[0]-robotPosition[0], 2) + pow(destination[1]-robotPosition[1], 2));
     printf("distanceToDestination: %.5f\n", distanceToDestination);
     moveForward(distanceToDestination);
     printf("Stop Coordinate: %.5f %.5f\n", robotPosition[0], robotPosition[1]);
     
}

int main(int argc, char **argv) {
   initializeRobot();
   
   // double destinationCoordinates[2] = {-0.000000, -0.0094};
   double destinationCoordinates[2] = {-0.258, -0.45};
   r->step(1);
   moveRobot(destinationCoordinates);
   delete r;
   return 0; //EXIT_SUCCESS
}
