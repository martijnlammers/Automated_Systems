import cv2
import cvzone
import sys


class Grid():
    def __init__(self, xGridSize = 0, yGridSize = 0, gridWidth = 0, gridHeight = 0, frameHeight = 0, frameWidth = 0):
        self.xGridSize = xGridSize
        self.yGridSize = yGridSize
        self.width = gridWidth
        self.height = gridHeight
        self.frameHeight = frameHeight
        self.frameWidth = frameWidth
        self.gridDefined = False

(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')

# Set up tracker.
# Instead of MIL, you can also use

tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN', 'MOSSE', 'CSRT']
tracker_type = tracker_types[7] #currently using CSRT since it seems to behave the way we want it to

if int(minor_ver) < 3:
    tracker = cv2.Tracker_create(tracker_type)
else:
    if tracker_type == 'BOOSTING':
        tracker = cv2.TrackerBoosting_create()
    if tracker_type == 'MIL':
        tracker = cv2.TrackerMIL_create()
    if tracker_type == 'KCF':
        tracker = cv2.TrackerKCF_create()
    if tracker_type == 'TLD':
        tracker = cv2.TrackerTLD_create()
    if tracker_type == 'MEDIANFLOW':
        tracker = cv2.TrackerMedianFlow_create()
    if tracker_type == 'GOTURN':
        tracker = cv2.TrackerGOTURN_create()
    if tracker_type == 'MOSSE':
        tracker = cv2.TrackerMOSSE_create()
    if tracker_type == "CSRT":
        tracker = cv2.TrackerCSRT_create()


# Start of main code
# TODO: put into functions
grid = Grid()

# Read video
video = cv2.VideoCapture("C:\\Users\\rtsmo\\Downloads\\car-overhead-1_Slomo.mp4")

# Exit if video not opened.
if not video.isOpened():
    print ("Could not open video")
    sys.exit()

# Read first frame.
ok, frame = video.read()
if not ok:
    print ('Cannot read video file')
    sys.exit()



# Define an initial bounding box
bbox = (287, 23, 86, 320)

# Uncomment the line below to select a different bounding box
bbox = cv2.selectROI(frame, False)

# Initialize tracker with first frame and bounding box
ok = tracker.init(frame, bbox)
_, _, grid.width, grid.height = bbox

def drawGrid():
    gridY, gridX = (0,0)
    xGridSize = yGridSize = 0
    grid.frameHeight, grid.frameWidth, _ = frame.shape
    xSizeKnown = False
    gridCounter = 0

    for gridY in range(grid.height, grid.frameHeight, grid.height):
        for gridX in range(grid.width, grid.frameWidth, grid.width):
            
            cv2.rectangle(frame, (gridX- grid.width, gridY - grid.height), (gridX, gridY), (0,255,0), 2)
            cv2.putText(frame, f"{gridCounter}", (int(gridX - (grid.width/1.2)), int(gridY - (grid.height/2))), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
            gridX += grid.width
            gridCounter += 1
            if not xSizeKnown and not grid.gridDefined:
                grid.xGridSize += 1
        gridY += grid.height
        xSizeKnown = True
        
        if not grid.gridDefined:
            grid.yGridSize += 1
    grid.gridDefined = True

drawGrid()  #Always draw a grid to make sure it is initialized properly

