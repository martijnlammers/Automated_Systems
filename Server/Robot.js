var robots = [];
var robotCounter = 3;

class Robot {
    constructor(robotID){
      this.robotID = robotID;
      this.position = 0;
      this.positionReached = true;
      this.path = [];
      this.pathIndex = 0;
      this.targetSetIndex = 0;
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

module.exports = { robots, robotCounter, Robot, addRobot, removeRobot };

