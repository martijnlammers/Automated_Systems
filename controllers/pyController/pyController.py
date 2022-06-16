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
import uuid

# Constants

HOST, PORT = "145.24.222.37", 8005
USERNAME, PASSWORD = "robots", "robots"

TIME_STEP = 20
ROTATE_SPEED, MAX_SPEED = 1, 5
LEFT = 0
RIGHT = 1
POS_CORRECTION = 3
POS_MATCHING_ACC = 0.03
ANGLE_ACCURACY = 1

# Global variables.

# Robot sensors and actuators

r = Robot()
motors = [r.getDevice("wheel_left"), r.getDevice("wheel_right")]
gps_mid, gps_front = r.getDevice("middleGPS"), r.getDevice("frontGPS")
sensor_light, sensor_dist = r.getDevice("light sensor"), r.getDevice("distance sensor")
sensor_ground, sensor_inert = r.getDevice("ground_ds"), r.getDevice("inertial unit")

client = mqtt.Client()
robotId, uuid = None, uuid.uuid4().hex
action_index = 2
state = "NO_TASK"
found_target, obstacle = False, False
path = []

def initializeRobot():
    r.step(TIME_STEP)
    for motor in motors:
        motor.setPosition(float('inf'))
    gps_mid.enable(TIME_STEP)
    gps_front.enable(TIME_STEP)
    sensor_inert.enable(TIME_STEP)
    sensor_light.enable(TIME_STEP)
    sensor_dist.enable(TIME_STEP)
    sensor_ground.enable(TIME_STEP)
    motorStop()

# Basic movement for the robot.


def motorStop():
    motors[LEFT].setVelocity(0)
    motors[RIGHT].setVelocity(0)
    r.step(TIME_STEP)


def motorMoveForward():
    motors[LEFT].setVelocity(MAX_SPEED)
    motors[RIGHT].setVelocity(MAX_SPEED)
    r.step(TIME_STEP)


def motorRotateLeft():
    motors[LEFT].setVelocity(-ROTATE_SPEED)
    motors[RIGHT].setVelocity(ROTATE_SPEED)
    r.step(TIME_STEP)


def motorRotateRight():
    motors[LEFT].setVelocity(ROTATE_SPEED)
    motors[RIGHT].setVelocity(-ROTATE_SPEED)
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
    initialAngle = calculateAngleToTarget(getVectors(destination))
    newAngle = initialAngle

    # This block is used to determine to rotate
    # clockwise or counter clockwise.

    while(initialAngle == calculateAngleToTarget(getVectors(destination))):
        motorRotateLeft()
    motorStop()
    newAngle = calculateAngleToTarget(getVectors(destination))

    # Rotate wheels until the angle is close
    # to 0 deg.

    if(initialAngle > newAngle):
        motorRotateLeft()
        while(calculateAngleToTarget(getVectors(destination)) > ANGLE_ACCURACY):
            r.step(TIME_STEP)
       
    else:
        motorRotateRight()
        while(calculateAngleToTarget(getVectors(destination)) > ANGLE_ACCURACY):
            r.step(TIME_STEP)
    motorStop()
    return


def moveToTarget(destination):
    global found_target, obstacle

    # The condition variable is used to
    # constructs a do-while loop, which
    # don't exist in Python.
    motorMoveForward()
    condition = True
    while(condition and not found_target and not obstacle):
        vectors = getVectors(destination)
        lengthToDestination = abs(m.sqrt(m.pow(vectors[0][0], 2) + m.pow(vectors[0][1], 2)))
        found_target = True if (sensor_light.getValue() > 450) else False
        obstacle = True if (sensor_ground.getValue() ==
                            1000 or sensor_dist.getValue() < 400) else False
        condition = (lengthToDestination > POS_MATCHING_ACC)
        r.step(TIME_STEP)
    if(not found_target and not obstacle):
        for i in range(POS_CORRECTION):
            r.step(1)
    motorStop()
    if(found_target):
        print("found")
        # Send MQTT data
        # found_target = False
    elif(obstacle):
        print("obstacle")
        # Send MQTT data
        # obstacle = False
    return


def moveRobot(node_number):
    global path
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
    publishToServer("robots/toServer/" + robotId + "/position", translateToNodeNumber(gps_mid.getValues()))
    publishToServer("robots/toServer/" + robotId + "/heading", getHeading())
    publishToServer("robots/toServer/" + robotId + "/state", "NO_TASK")


# Position translators will turn x,y coordinates in corresponding node number
# and visa versa for server-robot interactions..
def translateToCoordinates(node_nr):
    return [((int(node_nr / 10)) * 0.2) - 0.1084682, ((node_nr % 10) * 0.2) - 0.002948]


def translateToNodeNumber(pos):
    return int(str(int(m.ceil((round(pos[0] + 0.1084682, 1)) / 0.2))) +
               str(int(m.ceil((round(pos[1] + 0.002948, 1)) / 0.2))))


# MQTT functions
def publishToServer(topic, message):
    global robotId, client
    client.publish(topic, message, 1)
    print(str(robotId) + " publishing to: " +
          str(topic) + " message: " + str(message))


def on_connect(client, userdata, flags, rc):
    print("Robot connected successfully, response code: " + str(rc))


def on_message(client, userdata, msg):
    global action_index, robotId, path, uuid
    action = str(msg.topic).split("/")[action_index]
    payload = str(msg.payload.decode("utf-8"))
    print(action + " " + payload + " " + uuid)

    if(action.__eq__("setPath")):
        next_positions = payload.split(',')
        for i in range(len(next_positions)):
            path.append(int(next_positions[i]))

    elif(action.__eq__("updatePath")):
        print("update")
    elif(action.__eq__("register")):
        robotId = payload
        client.subscribe("robots/toRobot/" + robotId + "/#")
        topics = ["/state", "/model", "/heading", "/position", "/begin"]
        messages = ["NO_TASK", "VIRTUAL", getHeading(
        ), translateToNodeNumber(gps_mid.getValues()), ""]
        for i in range(len(topics)):
            publishToServer(
                f"robots/toServer/{robotId}{topics[i]}", messages[i])

        # The MQTT format changes after registration
        # so the topic index also has to change.

        action_index = 3
    else:
        print("wat")
    return


def initializeMQTTClient():
    client.on_connect = on_connect
    client.on_message = on_message
    client.username_pw_set(USERNAME, PASSWORD)
    client.connect(HOST, PORT, 5)
    client.subscribe("robots/toRobot/register/" + uuid)
    client.publish("robots/toServer/register/" + uuid, "")


if __name__ == "__main__":
    initializeRobot()
    initializeMQTTClient()
    while(r.step(TIME_STEP) != -1):
        client.loop()
        if(len(path) > 0):
            moveRobot(path.pop(0))
            r.step(TIME_STEP)
