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
        self.gridPos = -1 #unknown?
        self.heading = -1 #unknown?
        self.center = getCenterOfTwoPoints(redPos, greenPos)


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

    print(f"grX: {gridPosX}, grY: {gridPosY}, grNum: {gridNum}")
    cv.circle(output, (xPos, yPos), 10, (0,255,255), cv.FILLED)

    drawGrid(output)

def drawCircles(img, name):
    # coords = cv.findNonZero(mask)
    # for coord in coords:
    #     cv.circle(img, coord[0], 100, (0,0,255), 10)
    
    # convert the image to grayscale
    gray_image = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # convert the grayscale image to binary image
    ret,thresh = cv.threshold(gray_image,127,255,0)

    # find contours in the binary image
    contours, hierarchy = cv.findContours(thresh,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
    for c in contours:
    # calculate moments for each contour
        M = cv.moments(c)

        # calculate x,y coordinate of center
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = 0, 0

        # put text and highlight the center
        #cv.circle(img, (50, 50), 500, (0, 0, 255), 15)
        #cv.putText(img, name, (cX - 25, cY - 25),cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv.circle(img, (cX, cY), 5, (255, 0, 0), 10)
        cv.putText(img, "centroid", (cX - 25, cY - 25),cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # display the image
        #cv.imshow("Image", img)
        #cv.waitKey(30000)


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
            
        # Draw a circle based centered at centroid coordinates
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

def updateRobots(distanceAndPoints):
    for robot in robots:
        distToClosestCenter = 9999
        closestCenter = (0,0)
        for redPos, greenPos, distance in distanceAndPoints:
            currentCenter = getCenterOfTwoPoints(redPos, greenPos)
            distanceBetweenCenters = distBetweenPoints(robot.center, currentCenter)
            if distanceBetweenCenters < distToClosestCenter and distance < MAX_DISTANCE:
                closestCenter = currentCenter
                distToClosestCenter = distanceBetweenCenters
        robot.center = closestCenter
        #TODO: TEST
            
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
    cap = cv.VideoCapture("C:\\Users\\rtsmo\\Downloads\\robotCarColor.mp4")
    firstFrame = True
    while(1):
        _, frame = cap.read()

        frame = cv.resize(frame, (800, 450))
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        mask = cv.inRange(hsv, lower_hsv, upper_hsv)
        mask2 = cv.inRange(hsv, lower_hsv2, upper_hsv2)
        mask3 = cv.inRange(hsv, lower_green_hsv, upper_green_hsv)

        output = cv.bitwise_and(frame, frame, mask = (mask+mask2+mask3))
        redMasked = cv.bitwise_and(frame, frame, mask = (mask+mask2))
        greenMasked = cv.bitwise_and(frame, frame, mask = (mask3))
        

        redPositions = getContours(output, redMasked, "red")
        greenPositions = getContours(output, greenMasked, "green")

        #creating robot objects TODO:
        #get all positions of red and green contours
        #sort them based on distance (distance should roughly be the same each time and for each pair)
        #create a robot instance for each pair of red/green objects
        
        #updating robot object locations and variables TODO:
        #get all positions of red/green contours
        #get the positions that are closest to the previous positions, this is the location of the new position

        #!!!CENTER OF THE TWO POINTS CLOSEST TO THE PREVIOUS CENTER WILL BE THE NEW POINTS
        
        distancesAndPoints = []

        if greenPositions and redPositions:
            for redPos in redPositions:
                for greenPos in greenPositions:
                    distance = distBetweenPoints(greenPos, redPos)
                    distancesAndPoints.append((redPos, greenPos, distance))
                    
            cv.line(output, greenPositions[0], redPositions[0], (0, 255, 255), 2)

            #create robot objects
            #in this stage it is important that the robots are spaced far enough apart from each other!
            robotAmount = min(len(greenPositions), len(redPositions))
        
            if firstFrame:
                firstFrame = False
                initRobots(distancesAndPoints, robotAmount)
            else:
                ctr = updateRobots(distancesAndPoints)

        drawGrid(output)
        for robot in robots:
            objPositionOnGrid(robot.center[0], robot.center[1])

        cv.imshow("frame", frame)
        cv.imshow("output", output)

        k = cv.waitKey(1) & 0xff
        if k == 27 : break


    cap.release()
    output.release()
    cv.destroyAllWindows()

else:
    frame = cv.imread("C:\\Users\\rtsmo\\Downloads\\colorTest.png")
    frame = cv.resize(frame, (1200, 675))
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    mask = cv.inRange(hsv, lower_hsv, upper_hsv)
    mask2 = cv.inRange(hsv, lower_hsv2, upper_hsv2)
    mask3 = cv.inRange(hsv, lower_green_hsv, upper_green_hsv)
    allMask = (mask+mask2+mask3)

    output = cv.bitwise_and(frame, frame, mask = mask3)

    thresh_img = cv.threshold(mask2, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)[1]

    cnts = cv.findContours(thresh_img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    getContours(frame, output)

    cv.imshow("frame", frame)
    cv.imshow("output", output)
    cv.imshow("countours", thresh_img)
    cv.waitKey(0)
