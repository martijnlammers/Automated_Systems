import cv2 as cv
import numpy as np
import math

video = 1

lower_hsv = np.array([0,50,50])
upper_hsv = np.array([4,255,255])

lower_hsv2 =  np.array([170,50,50])
upper_hsv2 =  np.array([179,255,255])

lower_green_hsv =  np.array([42,50,50])
upper_green_hsv =  np.array([75,255,255])

MAX_DISTANCE = 150 #change if camera distance is increased/decreased
RESOLUTION = (800, 450)

robots = []

class Robot:
    def __init__(self, redPos, greenPos):
        self.redPos = redPos
        self.greenPos = greenPos
        self.gridPos = 0 #unknown?
        self.heading = -1 #unknown?
        self.center = getCenterOfTwoPoints(redPos, greenPos)
        self.angleOffset = -30
        self.heading = 0
    
    def drawSelf(self, imgOut):
        cv.circle(imgOut, self.center, 5, (0, 255, 255), cv.FILLED)
        cv.line(imgOut, self.greenPos, self.redPos, (0, 255, 255), 2)
    
    def findHeading(self):
        self.heading = int((findAngleBetweenPoints(self.greenPos, self.redPos) + self.angleOffset)  % 360)

class Grid():

    # Default grid is 10x10 in size
    def __init__(self, frame, xGridSize = 10, yGridSize = 10):
        self.xGridSize = xGridSize
        self.yGridSize = yGridSize
        self.frameHeight, self.frameWidth, _ = frame.shape
        self.width = int(self.frameWidth/self.xGridSize)
        self.height = int(self.frameHeight/self.yGridSize)
    
    def positionOnGrid(self, xPos, yPos):
        gridPosX = int(xPos / self.width)
        gridPosY = int(yPos / self.height)
        gridNum = gridPosX + (gridPosY * self.xGridSize)
        return gridNum

    def draw(self, img):
        gridCounter = 0

        for gridY in range(self.height, self.frameHeight, self.height-1):
            for gridX in range(self.width, self.frameWidth, self.width-1):
                cv.rectangle(img, (gridX- self.width, gridY - self.height), (gridX, gridY), (0,255,0), 2)
                cv.putText(img, f"{gridCounter}", (int(gridX - (self.width/1.2)), int(gridY - (self.height/2))), cv.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
                gridCounter += 1

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
        # print(f"robot center: {robot.center}, current:{currentCenter}, {distanceBetweenCenters}, {distance}")
        
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


if(video):
    # cap = cv.VideoCapture("C:\\Users\\rtsmo\\Downloads\\robotCarColor.mp4")
    cap = cv.VideoCapture("C:\\Users\\rtsmo\\Downloads\\multipleRobotTest2.mp4")
    firstFrame = True

    while(1):
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

                    gridPos = grid.positionOnGrid(robot.center[0], robot.center[1])
                    robot.gridPos = gridPos
                    robot.drawSelf(processedImg)
        
        cv.imshow("frame", frame)
        cv.imshow("output", processedImg)

        # Press 'escape' to quit
        k = cv.waitKey(1) & 0xff
        if k == 27 : break


    cap.release()
    processedImg.release()
    cv.destroyAllWindows()

# Code to test with one image delete in final code
else:
    frame = cv.imread("C:\\Users\\rtsmo\\Downloads\\colorTest.png")
    frame = cv.resize(frame, RESOLUTION)
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    mask = cv.inRange(hsv, lower_hsv, upper_hsv)
    mask2 = cv.inRange(hsv, lower_hsv2, upper_hsv2)
    mask3 = cv.inRange(hsv, lower_green_hsv, upper_green_hsv)
    allMask = (mask+mask2+mask3)

    processedImg = cv.bitwise_and(frame, frame, mask = mask3)

    thresh_img = cv.threshold(mask2, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)[1]

    cnts = cv.findContours(thresh_img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    getContours(frame, processedImg)

    cv.imshow("frame", frame)
    cv.imshow("output", processedImg)
    cv.imshow("countours", thresh_img)
    cv.waitKey(0)
