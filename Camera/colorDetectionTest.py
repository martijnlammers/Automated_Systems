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

robots = []

class Robot:
    def __init__(self, redPos, greenPos):
        self.redPos = redPos
        self.greenPos = greenPos
        self.gridPos = 0 #unknown?
        self.heading = -1 #unknown?
        self.center = getCenterOfTwoPoints(redPos, greenPos)
    
    def drawSelf(self, imgOut):
        cv.circle(imgOut, self.center, 5, (0, 255, 255), cv.FILLED)
        cv.line(imgOut, self.greenPos, self.redPos, (0, 255, 255), 2)


class Grid():
    def __init__(self, xGridSize = 10, yGridSize = 10, gridWidth = 0, gridHeight = 0, frameHeight = 0, frameWidth = 0):
        self.xGridSize = xGridSize
        self.yGridSize = yGridSize
        self.width = gridWidth
        self.height = gridHeight
        self.frameHeight = frameHeight
        self.frameWidth = frameWidth
        self.gridDefined = False
grid = Grid()


def drawGrid(frame):
    gridY, gridX = (0,0)
    grid.frameHeight, grid.frameWidth, _ = frame.shape
    xSizeKnown = False
    gridCounter = 0
    grid.width = int(grid.frameWidth/grid.xGridSize)
    grid.height = int(grid.frameHeight/grid.yGridSize)

    for gridY in range(grid.height, grid.frameHeight, grid.height-1):
        for gridX in range(grid.width, grid.frameWidth, grid.width-1):
            
            cv.rectangle(frame, (gridX- grid.width, gridY - grid.height), (gridX, gridY), (0,255,0), 2)
            cv.putText(frame, f"{gridCounter}", (int(gridX - (grid.width/1.2)), int(gridY - (grid.height/2))), cv.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
            gridX += grid.width
            gridCounter += 1
        gridY += grid.height
        xSizeKnown = True
        
    grid.gridDefined = True

def objPositionOnGrid(xPos, yPos):
    gridPosX = int(xPos / grid.width)
    gridPosY = int(yPos / grid.height)
    gridNum = gridPosX + (gridPosY * grid.xGridSize)
    return gridNum

def getContours(originalImg, contourImg, color):
    posArray = []
    gray_image = cv.cvtColor(contourImg, cv.COLOR_BGR2GRAY)
    ret, thresh = cv.threshold(gray_image, 50, 255, cv.THRESH_BINARY)

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
                        cv.circle(originalImg, midpoint, 4, (0, 255, 0), -1)
                        posArray.append(midpoint)
                    elif color == "green":
                        cv.circle(originalImg, midpoint, 4, (0, 0, 255), -1)
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

        frame = cv.resize(frame, (800, 450))
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
                drawGrid(processedImg)
            else:
                drawGrid(processedImg)
                for robot in robots:
                    updateRobot(robot, distancesAndPoints)

                    gridPos = objPositionOnGrid(robot.center[0], robot.center[1])
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
    frame = cv.resize(frame, (1200, 675))
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
