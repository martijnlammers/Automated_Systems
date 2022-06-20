import cv2 as cv
import numpy as np
import math

video = 1

lower_hsv = np.array([0,50,50])
upper_hsv = np.array([4,255,255])

lower_hsv2 =  np.array([176,50,50])
upper_hsv2 =  np.array([179,255,255])

lower_green_hsv =  np.array([45,50,50])
upper_green_hsv =  np.array([70,255,255])

MAX_DISTANCE = 100 #change if camera distance is increased/decreased

robots = []

class Robot:
    def __init__(self, redPos, greenPos):
        self.redPos = redPos
        self.greenPos = greenPos
        self.gridPos = -1 #unknown?
        self.heading = -1 #unknown?
        self.center = getCenterOfTwoPoints(redPos, greenPos)



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
        if cv.contourArea(contour) > 150:
            goodContours.append(index)

    
    # Calculate image moments of the detected contour
        for i in goodContours:
            M = cv.moments(contours[i])        
            cv.drawContours(originalImg, contours, i, (255, 0, 0), 2)
            
        # Draw a circle based centered at centroid coordinates
            if M["m00"] != 0:
                if color == "red":
                    cv.circle(originalImg, (round(M['m10'] / M['m00']), round(M['m01'] / M['m00'])), 1, (0, 255, 0), -1)
                    posArray.append((round(M['m10'] / M['m00']), round(M['m01'] / M['m00'])))
                elif color == "green":
                    cv.circle(originalImg, (round(M['m10'] / M['m00']), round(M['m01'] / M['m00'])), 1, (0, 0, 255), -1)
                    posArray.append((round(M['m10'] / M['m00']), round(M['m01'] / M['m00'])))
                
            

if(video):
    cap = cv.VideoCapture("C:\\Users\\rtsmo\\Downloads\\robotCarColor.mp4")
    firstFrame = True
    while(1):
        _, frame = cap.read()
        frame = cv.resize(frame, (450, 800))
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        mask = cv.inRange(hsv, lower_hsv, upper_hsv)
        mask2 = cv.inRange(hsv, lower_hsv2, upper_hsv2)
        mask3 = cv.inRange(hsv, lower_green_hsv, upper_green_hsv)

        output = cv.bitwise_and(frame, frame, mask = (mask+mask2+mask3))
        greenMasked = cv.bitwise_and(frame, frame, mask = (mask+mask2))
        redMasked = cv.bitwise_and(frame, frame, mask = (mask3))

        redPositions = []
        greenPositions = []

        getContours(output, redMasked, "red", redPositions)
        getContours(output, greenMasked, "green", greenPositions)

        if len(greenPositions) > 0 and len(redPositions) > 0:
            cv.line(output, greenPositions[0], redPositions[0], (0, 255, 255), 2)

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
