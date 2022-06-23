import cv2 as cv
import numpy as np
import math

import paho.mqtt.client as mqtt

video = 1

lower_hsv = np.array([0,50,50])
upper_hsv = np.array([4,255,255])

lower_hsv2 =  np.array([170,50,50])
upper_hsv2 =  np.array([179,255,255])

lower_green_hsv =  np.array([42,50,50])
upper_green_hsv =  np.array([75,255,255])

MAX_DISTANCE = 150 #change if camera distance is increased/decreased
# RESOLUTION = (800, 450)
RESOLUTION = (1280, 720)

robotsStopped = True

robots = []


broker = '145.24.222.37'
port = 8005
client_id = 'TrackingClient1'
username = 'robots'
password = 'robots'

class Robot:
    def __init__(self, redPos, greenPos):
        self.robotID = None
        self.redPos = redPos
        self.greenPos = greenPos
        self.gridPos = 0
        self.heading = 0
        self.prevHeading = 0     #this is the average heading of the past 10 frames
        self.center = getCenterOfTwoPoints(redPos, greenPos)
        self.angleOffset = -30
    
    def drawSelf(self, imgOut):
        cv.circle(imgOut, self.center, 5, (0, 255, 255), cv.FILLED)
        cv.line(imgOut, self.greenPos, self.redPos, (0, 255, 255), 2)
    
    def findHeading(self):
        self.prevHeading = self.heading
        self.heading = int((findAngleBetweenPoints(self.greenPos, self.redPos) + self.angleOffset + 90)  % 360) 

    def sendPosition(self, client):
        topic = f"robots/toServer/{self.robotID}/position"
        publishMQTT(client, topic, self.gridPos)

    def sendHeading(self, client):
        topic = f"robots/toServer/{self.robotID}/heading"
        publishMQTT(client, topic, self.heading)

class Grid():

    # Default grid is 12x12 in size
    def __init__(self, frame, xGridSize = 12, yGridSize = 6):
        self.xGridSize = xGridSize
        self.yGridSize = yGridSize
        self.frameHeight, self.frameWidth, _ = frame.shape

        # offset in pixels depending on the distance and resolution of the camera
        self.pixelHeightOffset = 85

        self.width = int(self.frameWidth/self.xGridSize) 
        self.height = int((self.frameHeight - self.pixelHeightOffset)/self.yGridSize)
        
    def positionOnGrid(self, xPos, yPos):
        gridPosX = int(xPos / self.width)
        gridPosY = int(yPos / self.height)
        gridNum = gridPosX + (gridPosY * self.xGridSize)
        return gridNum

    def draw(self, img):
        gridCounter = 0

        for gridY in range(self.height, self.frameHeight, self.height):
            for gridX in range(self.width, self.frameWidth, self.width):
                cv.rectangle(img, (gridX- self.width, gridY - self.height), (gridX, gridY), (0,255,0), 1)
                cv.putText(img, f"{gridCounter}", (int(gridX - (self.width/1.2)), int(gridY - (self.height/2))), cv.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
                gridCounter += 1

def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT")
        else:
            print("Failed to connect to MQTT")
    #Set Connecting Client ID
    client = mqtt.Client(client_id)
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker,port)
    return client

def publishMQTT(client, topic, msg):
    result = client.publish(topic, msg)
    status = result[0]
    if status == 0:
        pass
        print(f"Send `{msg}` to topic `{topic}`")
    else:
        print(f"Failed to send message to topic {topic}")

def subscribeMQTT(client):
    def on_message(client, userdata, msg):
        text = msg.payload.decode()
        robotID = msg.topic # pick robotID from topic
        print(f"Received `{text}` from `{msg.topic}` topic")
        robot = getRobot(robotID)
        robot.sendHeading(client)

    client.subscribe(f"robots/toServer/+/rotateAck")
    client.on_message = on_message

def getRobot(searchId):
    for robot in robots:
        if(robot.robotID == searchId):
            return robot

def findAngleBetweenPoints(p1, p2):
    deltaX = p1[0] - p2[0]
    deltaY = p2[1] - p1[1]
    
    return (math.degrees(math.atan2(deltaX, deltaY)) + 180) % 360

def getContours(originalImg, contourImg, color):
    posArray = []
    gray_image = cv.cvtColor(contourImg, cv.COLOR_BGR2GRAY)
    _, thresh = cv.threshold(gray_image, 50, 255, cv.THRESH_BINARY)

    #find contours
    contours, hierarchy = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    #remove small contours
    goodContours = []
    for index, contour in enumerate(contours):
        if cv.contourArea(contour) > 60:
            goodContours.append(index)
    
    # Calculate image moments of the detected contour
        for i in goodContours:
            M = cv.moments(contours[i])
            cv.drawContours(originalImg, contours, i, (255, 0, 0), 2)
            
        # Draw a circle based centered at centroid coordinates and color
            if M["m00"] != 0:
                midpoint = (round(M['m10'] / M['m00']), round(M['m01'] / M['m00']))
                if midpoint not in posArray:
                    if color == "red":
                        cv.circle(originalImg, midpoint, 4, (0, 0, 255), -1)
                        posArray.append(midpoint)
                    elif color == "green":
                        cv.circle(originalImg, midpoint, 4, (0, 255, ), -1)
                        posArray.append(midpoint)
    if posArray is None:
        return list()
    return posArray
                
            
def initRobots(distancesAndPoints, robotAmount):
    distancesAndPoints.sort(key=lambda x: x[2])
    for i in range(robotAmount):
        robots.append(Robot(distancesAndPoints[i][0], distancesAndPoints[i][1]))

# Updates position of a robot
def updateRobot(robot, distanceAndPoints):
    distToClosestCenter = 9999
    closestCenter = (0,0)
    newRedPos = newGreenPos = (0,0)
    for redPos, greenPos, distance in distanceAndPoints:
        currentCenter = getCenterOfTwoPoints(redPos, greenPos)
        distanceBetweenCenters = distBetweenPoints(robot.center, currentCenter)

        if distanceBetweenCenters < distToClosestCenter and distance < MAX_DISTANCE:
            closestCenter = currentCenter
            newRedPos = redPos
            newGreenPos = greenPos
            distToClosestCenter = distanceBetweenCenters
    robot.center = closestCenter
    robot.redPos = newRedPos
    robot.greenPos = newGreenPos

    # get the heading
    robot.findHeading()

def distBetweenPoints(p1, p2):
    return math.sqrt(((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2))

def getCenterOfTwoPoints(p1, p2):
    width = abs(p1[0] - p2[0])
    height = abs(p1[1] - p2[1])
    
    xLeft = min(p1[0], p2[0])
    yTop = min(p1[1], p2[1])

    cX = int(xLeft + (width/2))
    cY = int(yTop + (height/2))
    return cX, cY

def linkRobotIdToRobot(robotID):
    if robotsStopped:
        for robot in robots:
            if robot.robotID is not None:
                deltaTheta = min(robot.heading - (robot.prevHeading), 360 - (robot.heading - (robot.prevHeading)))
                if abs(deltaTheta) > 3:
                    robot.robotID = robotID

def processVideo():
    # cap = cv.VideoCapture("C:\\Users\\rtsmo\\Downloads\\robotCarColor.mp4")
    cap = cv.VideoCapture("C:\\Users\\rtsmo\\Downloads\\multipleRobotTest2.mp4")
    # cap = cv.VideoCapture("C:\\Users\\inti1\\Videos\\Captures\\multipleRobotTest2.mp4")
    # cap = cv.VideoCapture(1)      # External cam
    firstFrame = True

    client = connect_mqtt()
    subscribeMQTT(client)
    while(1):
        client.loop_start()

        _, frame = cap.read()

        frame = cv.resize(frame, RESOLUTION)
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # Get masks of red and green
        mask = cv.inRange(hsv, lower_hsv, upper_hsv)
        mask2 = cv.inRange(hsv, lower_hsv2, upper_hsv2)
        mask3 = cv.inRange(hsv, lower_green_hsv, upper_green_hsv)

        processedImg = cv.bitwise_and(frame, frame, mask = (mask+mask2+mask3))
        redMasked = cv.bitwise_and(frame, frame, mask = (mask+mask2))
        greenMasked = cv.bitwise_and(frame, frame, mask = (mask3))
        
        redPositions = getContours(processedImg, redMasked, "red")
        greenPositions = getContours(processedImg, greenMasked, "green")

        grid = Grid(processedImg)
        
        # Get all positions and distances of red and green contours
        distancesAndPoints = []

        if greenPositions and redPositions:
            for redPos in redPositions:
                for greenPos in greenPositions:
                    distance = distBetweenPoints(greenPos, redPos)
                    distancesAndPoints.append((redPos, greenPos, distance))

            robotAmount = min(len(greenPositions), len(redPositions))

            # Initialize or update the robots with their positions
            if firstFrame:
                firstFrame = False
                initRobots(distancesAndPoints, robotAmount)
                for robot in robots:
                    robot.drawSelf(processedImg)
                grid.draw(processedImg)
            else:
                grid.draw(processedImg)
                for robot in robots:
                    updateRobot(robot, distancesAndPoints)

                    newGridPos = grid.positionOnGrid(robot.center[0], robot.center[1])
                    if(newGridPos != robot.gridPos):
                        robot.gridPos = newGridPos
                        robot.sendPosition(client)
                    robot.drawSelf(processedImg)

                    if robot.heading != robot.prevHeading:
                        robot.sendHeading(client)
                    
                    
        
        cv.imshow("frame", frame)
        cv.imshow("output", processedImg)

        # Press 'escape' to quit
        k = cv.waitKey(0) & 0xff
        if k == 27 : break


    cap.release()
    processedImg.release()
    cv.destroyAllWindows()

def main():
    processVideo()

if __name__ == "__main__":
    main()