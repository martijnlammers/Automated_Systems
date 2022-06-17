var robots = [];
var robotCounter = 0;
var targetSetIndex = 0;
var readyForNextTarget = 0;
const NUM_OF_ROBOTS = 2; // Can not be bigger than algorithm.rows

class Robot {
    constructor(robotID){
      this.robotID = robotID;
      this.position = 0;
      this.path = [];
      this.pathIndex = 0;
      this.model = "PHYSICAL";
      this.heading = 0;
      this.state = "NO_TASK";
      this.obstacleDetected = [false, false, false, false];
      this.goalDetected = false;
    }
}

function addRobot(robotID){
    newRobot = new Robot(robotID);
    robots.push(newRobot);
    return newRobot;
  }

function removeRobot(robotID){
    robots = robots.filter(robot => robot.robotID !== robotID);
}

module.exports = { robots, robotCounter, Robot, addRobot, removeRobot, NUM_OF_ROBOTS, targetSetIndex, readyForNextTarget};

