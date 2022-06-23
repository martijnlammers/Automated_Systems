# Automated Systems Webots simulation controller.
# Martijn Lammers


# Notable bugs:
# - If the robot doesn't reach the destination withing the positional accuracy,
# it will get angry and decide to run from the problem, just like an upset adolecent.
# If this happens, the constant POS_MATCHING_ACC needs to be higher.
# (Increase in increments of 0.005)

from controller import Robot, Motor, GPS, LightSensor, DistanceSensor, InertialUnit, Supervisor
import math as m
import paho.mqtt.client as mqtt
import uuid, time

# Constants

HOST, PORT = "145.24.222.37", 8005
USERNAME, PASSWORD = "robots", "robots"

TIME_STEP = 20
ROTATE_SPEED, MAX_SPEED = 1, 4
POS_CORRECTION = 5
POS_MATCHING_ACC = 0.03
ANGLE_ACCURACY = 1

# Global variables.

# Robot sensors and actuators
r = Robot()
motor_left = r.getDevice("wheel_left")
motor_right = r.getDevice("wheel_right")
gps_mid, gps_front = r.getDevice("middleGPS"), r.getDevice("frontGPS")
sensor_light, sensor_dist = r.getDevice("light sensor"), r.getDevice("distance sensor")
sensor_ground, sensor_inert = r.getDevice("ground_ds"), r.getDevice("inertial unit")

client = mqtt.Client()
robotId, uuid = None, uuid.uuid4().hex
action_index = 2
state = "NO_TASK"
destination = None
found_target = False
path = []

def setupRobot():
    r.step(TIME_STEP)
    motor_left.setPosition(float('inf'))
    motor_right.setPosition(float('inf'))
    gps_mid.enable(TIME_STEP)
    gps_front.enable(TIME_STEP)
    sensor_inert.enable(TIME_STEP)
    sensor_light.enable(TIME_STEP)
    sensor_dist.enable(TIME_STEP)
    sensor_ground.enable(TIME_STEP)
    motorStop()

# Basic movement for the robot.


def motorStop():
    motor_left.setVelocity(0)
    motor_right.setVelocity(0)
    r.step(TIME_STEP)


def motorMoveForward():
    motor_left.setVelocity(MAX_SPEED)
    motor_right.setVelocity(MAX_SPEED)
    r.step(TIME_STEP)


def motorRotateLeft():
    motor_left.setVelocity(-ROTATE_SPEED)
    motor_right.setVelocity(ROTATE_SPEED)
    r.step(TIME_STEP)


def motorRotateRight():
    motor_left.setVelocity(ROTATE_SPEED)
    motor_right.setVelocity(-ROTATE_SPEED)
    r.step(TIME_STEP)

# Math source: https://www.cuemath.com/geometry/angle-between-vectors/
# Calculates the rotation to the target.


def calculateAngleToTarget(vectors):
    a, b = vectors[0][0], vectors[0][1]
    c, d = vectors[1][0], vectors[1][1]
    dotProduct = ((a * c) + (b * d))
    return m.acos(dotProduct /
                  (m.sqrt(m.pow(a, 2) + m.pow(b, 2))
                   * m.sqrt(m.pow(c, 2) + m.pow(d, 2)))) \
        * 57.2957795
    # ^ Converting radians to degrees by multiplying by 57.29...


def getVectors(destination):
    m_robotPos = gps_mid.getValues()
    f_robotPos = gps_front.getValues()

    # Calculates two vectors;
    # Vector 1: Target(x,y) to MiddleRobot(x,y)
    # Vector 2: MiddleRobot(x,y) to FrontRobot(x,y)

    vectors = [[(destination[0] - m_robotPos[0]), (destination[1] - m_robotPos[1])],
               [(f_robotPos[0] - m_robotPos[0]), (f_robotPos[1] - m_robotPos[1])]]

    return vectors


def getHeading():
    r.step(TIME_STEP)
    deg = sensor_inert.getRollPitchYaw()[2] * 57.2957795
    deg = abs(deg) if (deg < 0) else 360 - deg

    # The algorithm on the server can only use
    # 0, 90, 180 and 270 to do it's calculations with.

    # The dictionary keeps track of the conditions and
    # corresponding value, substituting the need of an
    # if else block, reducing lines needed.

    headingDictionary = {(deg > 315 or deg <= 45): 180, (deg > 45 and deg <= 135): 270,
                         (deg > 135 and deg <= 225): 0, (deg > 225 and deg <= 315): 90}

    # dict.get(1) will always get the k:value (heading) of which
    # condition applies to the current degrees.

    return headingDictionary.get(1)


def rotateToTarget(destination):
    
    # This block is used to determine to rotate
    # clockwise or counter clockwise.
    
    initialAngle = calculateAngleToTarget(getVectors(destination))
    motorRotateLeft()
    r.step(TIME_STEP)
    motorStop()
    
    # Rotate wheels until the angle is close
    # to 0 deg.

    if(initialAngle > calculateAngleToTarget(getVectors(destination))):
        motorRotateLeft() 
    else:
        motorRotateRight()
    while(calculateAngleToTarget(getVectors(destination)) > ANGLE_ACCURACY):
        r.step(TIME_STEP)
    motorStop()
    return


def moveToTarget(destination):
    global path, found_target, on_target_pos, client
    obstacle = False
    
    # The condition variable is used to
    # constructs a do-while loop, which
    # don't exist in Python.
    
    condition = True
    motorMoveForward()
    while(condition):
        vectors = getVectors(destination)
        lengthToDestination = abs(m.sqrt(m.pow(vectors[0][0], 2) + m.pow(vectors[0][1], 2)))
        # found_target = True if (sensor_light.getValue() > 450) else False
        # obstacle = True if (sensor_ground.getValue() == 1000 or sensor_dist.getValue() < 400) else False
        if(sensor_dist.getValue() < 600):
            motorStop()
            updateRobotData()
            path = []
            publish(f"robots/toServer/{robotId}/robotDetected", "1,0,0,0")     
            time.sleep(0.5)
            return        
        if(sensor_ground.getValue() == 1000):
            motorStop()
            updateRobotData()
            path = []
            publish(f"robots/toServer/{robotId}/obstacleDetected", "1,0,0,0")
            return
        condition = (lengthToDestination > POS_MATCHING_ACC)
        r.step(TIME_STEP)
    if(not found_target and not obstacle):
        for i in range(POS_CORRECTION):
            r.step(1)
    motorStop()
    
    # Sends location and heading to broker 
    # once completed with task.
    
    updateRobotData()
    if(found_target):
        path = []
        publish(f"robots/toServer/{robotId}/goalDetected", "true")
        
def moveRobot(node_number):
    global path, destination
    destination = translateToCoordinates(node_number)
    m_robotPos = gps_mid.getValues()
    
    # Quits function when the destination equals the current position.

    if (m.fabs(m_robotPos[0] - destination[0]) < POS_MATCHING_ACC and
            m.fabs(m_robotPos[1] - destination[1]) < POS_MATCHING_ACC):
        return
    vectors = getVectors(destination)

    # Move robot to (x,y)
    rotateToTarget(destination)
    moveToTarget(destination)
    
    


# Position translators will turn x,y coordinates in corresponding node number
# and visa versa for server-robot interactions..
def translateToCoordinates(node_nr):
    return [((int(node_nr / 12)) * 0.2) - 0.0906478, ((node_nr % 12) * 0.2) + 0.0102571]


def translateToNodeNumber(pos):   
    return((m.ceil((round(pos[0]  + 0.1084682, 1)) / 0.2) * 12) + 
                (m.ceil((round(pos[1] + 0.002948, 1)) / 0.2)))


# MQTT functions
def publish(topic, message):
    global robotId, client
    client.publish(topic, message, qos=2)
    # print(str(robotId) + " publishing to: " +
          # str(topic) + " message: " + str(message))
def updateRobotData():
    publish("robots/toServer/" + robotId + "/position", str(translateToNodeNumber(gps_mid.getValues())))
    publish("robots/toServer/" + robotId + "/heading", getHeading())
    publish("robots/toServer/" + robotId + "/state", "NO_TASK")

def on_connect(client, userdata, flags, rc):
    print("Robot connected successfully, response code: " + str(rc))


def on_message(client, userdata, msg):
    global action_index, robotId, path, uuid, found_target
    action = str(msg.topic).split("/")[action_index]
    payload = str(msg.payload.decode("utf-8"))
    # print(action + " " + payload + " " + uuid)

    if(action.__eq__("path")):
        path = []   
        next_positions = payload.split(',')
        for i in range(len(next_positions)):
            path.append(int(next_positions[i]))
    elif(action.__eq__("register")):
        robotId = payload
        client.subscribe(f"robots/toRobot/{robotId}/#")
        topics = ["/state", "/model", "/heading", "/position", "/begin"]
        messages = ["NO_TASK", "VIRTUAL", getHeading(
        ), str(translateToNodeNumber(gps_mid.getValues())), ""]
        for i in range(len(topics)):
            publish(
                f"robots/toServer/{robotId}{topics[i]}", messages[i])

        # The MQTT format changes after registration
        # so the topic index also has to change.

        action_index = 3
    elif(action.__eq__("stop")):
        motorStop()
        path = []
    elif(action.__eq__("goalDetected")):
        target_found = True
    elif(action.__eq__("end")):
        path = []
        client.unsubscribe(f"robots/toRobot/register/{uuid}")
        client.unsubscribe(f"robots/toRobot/{robotId}/#")
    return


def setupMQTT():
    client.on_connect = on_connect
    client.on_message = on_message
    client.username_pw_set(USERNAME, PASSWORD)
    client.connect(HOST, PORT, 5)
    client.subscribe("robots/toRobot/register/" + uuid)
    client.publish("robots/toServer/register/" + uuid, "")

if __name__ == "__main__":
    setupRobot()
    setupMQTT()
    while(r.step(TIME_STEP) != -1):
        client.loop()
        # print(f"{robotId} {path}")
        if(len(path) > 0):
            dest = path.pop(0)
            moveRobot(dest)
        # else:
        #     client.publish("robots/toServer/{robotId}/getNextSet", "")
        r.step(TIME_STEP)
    # print(translateToNodeNumber(gps_mid.getValues()))
    # moveRobot(0)


""" 
robotDetected = false
robotDetectedSend = false

loop:
    if(robotDetected && robotDetectedSend == false):
        client.publish("robots/toServer/" +robotId+ "/robotDetected", "1,0,0,0")
        robotDetectedSend = true
        motorStop()  
    
    ReadIR()
    
subscribe toRobot/ID/drive => drive()
subscribe toServer/ID/heading => changeHeading()

ReadIR():
    if(IR):
        robotDetected = true

"""