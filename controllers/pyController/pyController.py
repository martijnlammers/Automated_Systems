# Automated Systems Webots simulation controller.
# Martijn Lammers


# Notable bugs:
# - If the robot doesn't reach the destination withing the positional accuracy,
# it will get angry and decide to run from the problem, just like an upset adolecent. 
# If this happens, the constant POS_MATCHING_ACC needs to be higher. 
# (Increase in increments of 0.005)

from controller import Robot, Motor, GPS, LightSensor, DistanceSensor, InertialUnit
import math as m
import paho.mqtt.client as mqtt
import uuid
import json

# Don't touch these preferably.
# Constants and variables.

HOST = "145.24.222.37"
PORT = 8005
USERNAME = "robots"
PASSWORD = "robots"
TIME_STEP = 32
ROTATE_SPEED = 0.07
MAX_SPEED = 8
LEFT = 0
RIGHT = 1
NUM_MOTORS = 2
ACTION = 2
POS_MARGIN = 0.1
POS_MATCHING_ACC = 0.07
ANGLE_ACCURACY = 0.000000001

# Global variables.
robot = Robot()
motors = [robot.getDevice("wheel_left"), robot.getDevice("wheel_right")];
m_gps = robot.getDevice("middleGPS")
f_gps = robot.getDevice("frontGPS")
lightsensor = robot.getDevice("light sensor")
distancesensor = robot.getDevice("distance sensor")
groundsensor = robot.getDevice("ground_ds")
inertialunit = robot.getDevice("inertial unit")
destinationCoordinates = [-0.4, -0.13]
client = mqtt.Client()
uuid = uuid.uuid4().hex
robotId = None
state = "NO_TASK" # | PROCESSING | 
found_target = False
obstacle = False


# Set up of sensors and actuators used by the robot.

def initializeRobot():
    f_gps.enable(TIME_STEP)
    m_gps.enable(TIME_STEP)
    lightsensor.enable(TIME_STEP)
    distancesensor.enable(TIME_STEP)
    groundsensor.enable(TIME_STEP)
    inertialunit.enable(TIME_STEP)
    motors[LEFT].setPosition(float('inf'))
    motors[RIGHT].setPosition(float('inf'))
    motorStop()
    
# Basic movement for the robot.

def motorStop():
    motors[LEFT].setVelocity(0)
    motors[RIGHT].setVelocity(0)
    robot.step(1)

def motorMoveForward():
    motors[LEFT].setVelocity(MAX_SPEED)
    motors[RIGHT].setVelocity(MAX_SPEED)
    robot.step(1)

def motorRotateLeft(speed_multiplier):
    motors[LEFT].setVelocity(speed_multiplier * -ROTATE_SPEED)
    motors[RIGHT].setVelocity(speed_multiplier * ROTATE_SPEED)
    robot.step(1)

def motorRotateRight(speed_multiplier):
    motors[LEFT].setVelocity(speed_multiplier * ROTATE_SPEED)
    motors[RIGHT].setVelocity(speed_multiplier * -ROTATE_SPEED)
    robot.step(1)

# Math source: https://www.cuemath.com/geometry/angle-between-vectors/
# Calculates the rotation to the target.
  
def calculateAngleToTarget(vectors):
    a, b = vectors[0][0], vectors[0][1]
    c, d = vectors[1][0], vectors[1][1]
    dotProduct = ((a * c) + (b * d));
    return m.acos(dotProduct / \
    (m.sqrt(m.pow(a, 2) + m.pow(b,2)) \
    * m.sqrt(m.pow(c, 2) + m.pow(d,2)))) \
    * 57.2957795;
    # ^ Converting radians to degrees by multiplying by 57.29...
    
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
    global found_target, obstacle
    # The condition variable is used to 
    # constructs a do-while loop, which 
    # don't exist in Python.
    
    condition = True
    
    while(condition and not found_target and not obstacle):
        vectors = getVectors(destination)
        a, b = vectors[0][0], vectors[0][1]   
        length = abs(m.sqrt(m.pow(a, 2) + m.pow(b,2))) - POS_MARGIN
        motorMoveForward()
        found_target = True if (lightsensor.getValue() > 450) else False
        
        obstacle = True if (groundsensor.getValue() == 1000 or distancesensor.getValue() <= 400) else False 
        condition = (length > POS_MATCHING_ACC)  
    motorStop()
    if(found_target):
        print("found")
        pass
        # Send MQTT data
        # found_target = False
    elif(obstacle):
        print("obstacle")
        pass
        # Send MQTT data 
        # obstacle = False
    return


def moveRobot(destination):
     robot.step(1)
     m_robotPos = m_gps.getValues()
     
     # Quits function when the destination equals the current position.
     
     if (m.fabs(m_robotPos[0] - destination[0]) < POS_MATCHING_ACC and  \
     m.fabs(m_robotPos[1] - destination[1]) < POS_MATCHING_ACC): 
         return
         
     vectors = getVectors(destination)
     
     # Move robot to (x,y) 
     
     rotateToTarget(destination)
     moveToTarget(destination)

def translateCoordinates(coordinates):
    return [(((coordinates[0])/100) - 1.67),\
    (((coordinates[1])/100 )-2.80)]
    
# MQTT functions
def publishToServer(client, topic, message):
    global robotId
    client.publish(topic, message, 1)
    print(str(robotId) + " publishing to: " + str(topic) + " message: " + str(message))
    
def on_connect(client, userdata, flags, rc):
    print("Robot connected successfully, response code: " + str(rc)) 
 
def on_message(client, userdata, msg):
    global ACTION, robotId
    topics = str(msg.topic).split("/")
    action = topics[ACTION]
    payload = str(msg.payload.decode("utf-8"))
    print(action + " " + payload + " " + uuid)
    
    if(action.__eq__("update")):
        publishToServer(client, "robots/toServer/" + robotId + "/position", 1)
        publishToServer(client, "robots/toServer/" + robotId + "/heading", 0)
    elif(action.__eq__("register")):
        robotId = payload
        client.unsubscribe("robots/toRobot/register/" + uuid)
        client.subscribe("robots/toRobot/" + robotId + "/#")
        publishToServer(client, "robots/toServer/" + robotId + "/state", "NO_TASK")
        publishToServer(client, "robots/toServer/" + robotId + "/model", "VIRTUAL")
        publishToServer(client, "robots/toServer/" + robotId + "/begin", "")
        # Position given should be dynamicly obtained, not hardcoded. 
        # Get x,y pos, check in which 
        publishToServer(client, "robots/toServer/" + robotId + "/position", 1)
     
        # The MQTT format changes after registration
        # so the topic index also has to change.
        
        ACTION = 3
    else:
        print("wat")
    
    
    
    
    
def initializeMQTTClient():
    client.on_connect = on_connect
    client.on_message = on_message
    client.username_pw_set(USERNAME, PASSWORD)
    client.connect(HOST, PORT, 5)
    client.subscribe("robots/toRobot/register/" + uuid)
    client.publish("robots/toServer/register/" + uuid, "")

if __name__ == "__main__":
    initializeRobot()
    #initializeMQTTClient()
    while(robot.step(TIME_STEP) != -1):
        #client.loop()
        deg = inertialunit.getRollPitchYaw()[2] * 57.2957795
        if(deg < 0):
            deg = abs(deg)
        elif(deg > 0):
            deg = 360 - deg
        print(deg)
     
        robot.step(1)
           
    #moveRobot(translateCoordinates([0,399]))

 
 
     # TODO:
         # GET HEADING ->CLOCKWISE
         # CHANGE GRID ->
             # hoegroot is een node
             # hoe groot het grid word
         # 13 x 11
        


