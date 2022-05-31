# Automated Systems Webots simulation controller.
# Martijn Lammers


# Notable bugs:
# - If the robot doesn't reach the destination withing the positional accuracy,
# it will get angry and decide to run from the problem, just like an upset adolecent. 
# If this happens, the constant POS_MATCHING_ACC needs to be higher. 
# (Increase in increments of 0.005)

from controller import Robot, Motor, GPS, LightSensor
import math

# Constants and variables.
TIME_STEP = 64
ROTATE_SPEED = 0.07
MAX_SPEED = 18
LEFT = 0
RIGHT = 1
NUM_MOTORS = 2

# Don't touch these preferably.
# It will mess up the movements.

POS_MATCHING_ACC = 0.005
ANGLE_ACCURACY = 0.001

robot = Robot()
motors = [robot.getDevice("wheel_left"), robot.getDevice("wheel_right")];
m_gps = robot.getDevice("middleGPS")
f_gps = robot.getDevice("frontGPS")
lightsensor = robot.getDevice("light sensor")
found_target = False
destinationCoordinates = [-0.4, -0.13]

# Basic movement for the robot.

def motorStop():
    motors[LEFT].setVelocity(0)
    motors[RIGHT].setVelocity(0)
    robot.step(1)

def motorMoveForward(speed_multiplier):
    motors[LEFT].setVelocity(speed_multiplier * MAX_SPEED)
    motors[RIGHT].setVelocity(speed_multiplier * MAX_SPEED)
    robot.step(1)

def motorRotateLeft(speed_multiplier):
    motors[LEFT].setVelocity(speed_multiplier * -ROTATE_SPEED)
    motors[RIGHT].setVelocity(speed_multiplier * ROTATE_SPEED)
    robot.step(1)

def motorRotateRight(speed_multiplier):
    motors[LEFT].setVelocity(speed_multiplier * ROTATE_SPEED)
    motors[RIGHT].setVelocity(speed_multiplier * -ROTATE_SPEED)
    robot.step(1)

# Set up of sensors and actuators used by the robot.

def initializeRobot():
    f_gps.enable(TIME_STEP)
    m_gps.enable(TIME_STEP)
    lightsensor.enable(TIME_STEP)
    motors[LEFT].setPosition(float('inf'))
    motors[RIGHT].setPosition(float('inf'))
    motorStop()

# Math source: https://www.cuemath.com/geometry/angle-between-vectors/
# Calculates the rotation to the target.
  
def calculateAngleToTarget(vectors):
    a, b = vectors[0][0], vectors[0][1]
    c, d = vectors[1][0], vectors[1][1]
    dotProduct = ((a * c) + (b * d));
    return math.acos(dotProduct / \
    (math.sqrt(math.pow(a, 2) + math.pow(b,2)) \
    * math.sqrt(math.pow(c, 2) + math.pow(d,2)))) \
    * 57.2957795;
    
def getVectors(destination):
    m_robotPos = m_gps.getValues()
    f_robotPos = f_gps.getValues()
  
    # Calculates two vectors;
    # Vector 1: Target(x,y) to MiddleRobot(x,y)
    # Vector 2: MiddleRobot(x,y) to FrontRobot(x,y)

    vectors = [[(destination[0] - m_robotPos[0]), (destination[1] - m_robotPos[1])], \
    [(f_robotPos[0] - m_robotPos[0]), (f_robotPos[1] - m_robotPos[1])]] 
    return vectors
    
    
 
def rotateToTarget(destination):
    initialAngle = calculateAngleToTarget(getVectors(destination));
    newAngle = initialAngle
    
    # This block is used to determine to rotate 
    # clockwise or counter clockwise.
     
    while(initialAngle == calculateAngleToTarget(getVectors(destination))):
      motorRotateLeft(newAngle);
    motorStop();
    
    # Rotate wheels until the angle is close 
    # to 0 deg.

    if(initialAngle > newAngle):
        while(newAngle > ANGLE_ACCURACY):
            motorRotateLeft(newAngle)
            newAngle = calculateAngleToTarget(getVectors(destination))
    else:
        while(newAngle > ANGLE_ACCURACY):
            motorRotateRight(newAngle)
            newAngle = calculateAngleToTarget(getVectors(destination))
      
    motorStop()
    return

def moveToTarget(destination):
    
    # The condition variable is used to 
    # constructs a do-while loop, which 
    # don't exist in Python.
    
    condition = True
    while(condition):
        vectors = getVectors(destination)
        a, b = vectors[0][0], vectors[0][1]   
        length = math.sqrt(math.pow(a, 2) + math.pow(b,2))
        motorMoveForward(length)
        condition = (length > POS_MATCHING_ACC)   
    motorStop()
    return


def moveRobot(destination):
     robot.step(1)
     m_robotPos = m_gps.getValues()
     
     # Quits function when the destination equals the current position.
     
     if (math.fabs(m_robotPos[0] - destination[0]) < POS_MATCHING_ACC and  \
     math.fabs(m_robotPos[1] - destination[1]) < POS_MATCHING_ACC): 
         return
         
     vectors = getVectors(destination)
     
     # Move robot to (x,y) 
     
     rotateToTarget(destination)
     moveToTarget(destination)



if __name__ == "__main__":
    print("Hello, World!")
    robot.step(1)
    initializeRobot()
    moveRobot(destinationCoordinates)


