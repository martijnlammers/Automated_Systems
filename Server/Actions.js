const communication = require("./Communication");
const robotLib = require("./Robot");
const algorithms = require("./Controller");
const munkres = require("munkres-js");

const ANGLE_MARGIN = 3;
const MOTOR_SPEED = 60;
var visitedLocations = new Array(100).fill(false);

// CHECK VARIABLE AND FUNCTION NAMES
function getVerticesAroundLocation(location) {
    let vertexAboveLocation = location - algorithms.columns;
    let vertexBelowLocation = location + algorithms.columns;
    let vertexLeftOfLocation = location - 1;
    let vertexRightOfLocation = location + 1;
    
    return [vertexAboveLocation, vertexRightOfLocation, vertexBelowLocation, vertexLeftOfLocation];
}

function shiftArray(array, heading, shiftRelative) {
    let angleBetweenVertices = 90;
    let amountToShift = heading / angleBetweenVertices;

    if (amountToShift == 0) {
        return array;
    } 

    if (shiftRelative) {
        for (let amountShifted = 0; amountShifted < amountToShift; amountShifted++) {
            array.push(array.shift());// SHIFT LEFT
        }
    } else {
        for (let amountShifted = 0; amountShifted < amountToShift; amountShifted++) {
            array.unshift(array.pop()); // SHIFT RIGHT
        }
    }

    return array;
}

function getObstacleLocations(robotLocation, heading, obstacleDetectedList) {
    let verticesAroundLocation = getVerticesAroundLocation(robotLocation);
    let verticesAroundLocationRelative = shiftArray(verticesAroundLocation, heading, true);
    let obstaclePositions = [];

    for (index in obstacleDetectedList) {
        if (obstacleDetectedList[index] == 1) {
            obstaclePositions.push(verticesAroundLocationRelative[index]);
        }
    }

    return obstaclePositions;
}

function getRobotPositions() {
    let robotPositions = [];

    for (robotIndex in robotLib.robots) {
        robotPositions.push(robotLib.robots[robotIndex].position);
    }

    return robotPositions;
}

function updatePaths(adjacencyMatrix, targetSet) {
    let startVertexVector = getRobotPositions();
    let pathAndAssingment = algorithms.getPathAndAssignment(adjacencyMatrix, algorithms.rows * algorithms.columns, startVertexVector, targetSet);

    let shortestPathMatrixRow, shortestPathMatrixColumn;

    optimalAssignment = munkres(pathAndAssingment[1]);
    // console.log("optimalAssigment", optimalAssignment);
    shortestPathMatrix = pathAndAssingment[0];
    
    for (robotIndex in robotLib.robots) {
        shortestPathMatrixRow = optimalAssignment[robotIndex][0];
        shortestPathMatrixColumn = optimalAssignment[robotIndex][1];
        robotLib.robots[robotIndex].path = shortestPathMatrix[shortestPathMatrixRow][shortestPathMatrixColumn];
        robotLib.robots[robotIndex].pathIndex = 1;
    }
}

function removeObstaclesFromAdjacencyMatrix(adjacencyMatrix, obstaclePosition) {
    // console.log("removing", obstaclePosition);
    adjacencyMatrix[obstaclePosition].fill(0);
    
    for (row in adjacencyMatrix) {
        adjacencyMatrix[row][obstaclePosition] = 0;
    }
}

function getUpdatedAdjacencyMatrix(obstacleLocations, robotPositions) {
    // let adjacencyMatrixCopy = JSON.parse(JSON.stringify(algorithms.adjacencyMatrix));
    
    for (obstacleIndex in obstacleLocations) { 
        let obstaclePosition = obstacleLocations[obstacleIndex]; 
        // console.log("obstaclePosition", obstaclePosition);
        if (robotPositions.includes(obstaclePosition)) continue
        else removeObstaclesFromAdjacencyMatrix(algorithms.adjacencyMatrix, obstaclePosition);
    }

    return algorithms.adjacencyMatrix;
}

function robotDetected(robot) {
    console.log(`${robot.robotID} has run into a robot!`);
    communication.publishMessage(`robots/toRobot/${robot.robotID}/stop`, "");
    target = robot.path.pop();
    console.log(robot.robotID, target, robot.position);
    let adjacencyMatrixCopy = JSON.parse(JSON.stringify(algorithms.adjacencyMatrix));
    detectedRobots = getObstacleLocations(robot.position, robot.heading, robot.obstacleDetected);
    console.log(detectedRobots);
    detectedRobots.forEach((detectedRobot) => {
        removeObstaclesFromAdjacencyMatrix(adjacencyMatrixCopy, detectedRobot);
    });
    // 6. Hij herberekent het pad
    previous = algorithms.breadthFirstSearch(adjacencyMatrixCopy, robot.position, algorithms.rows * algorithms.columns);
    path = algorithms.traceRoute(previous, robot.position, target);
    // 7. Geeft pad aan de robot
    robot.path = path;
    robot.pathIndex = 1;

    if(robot.model === "VIRTUAL"){
        communication.publishMessage(`robots/toRobot/${robot.robotID}/path`, `${robot.path}`);
    }

}

function obstacleDetected(robot) {
    communication.publishMessage(`robots/toRobot/${robot.robotID}/stop`, "");
    target = robot.path.pop();
    detectedObstacles = getObstacleLocations(robot.position, robot.heading, robot.obstacleDetected);
    console.log(detectedObstacles);
    detectedObstacles.forEach((detectedObstacle) => {
        removeObstaclesFromAdjacencyMatrix(algorithms.adjacencyMatrix, detectedObstacle);
        visitedLocations[detectedObstacle] = true;
    });
    previous = algorithms.breadthFirstSearch(algorithms.adjacencyMatrix, robot.position, algorithms.rows * algorithms.columns);
    path = algorithms.traceRoute(previous, robot.position, target);
    robot.path = path;
    robot.pathIndex = 1;

    if(robot.model === "VIRTUAL"){
        communication.publishMessage(`robots/toRobot/${robot.robotID}/path`, `${robot.path}`);
    }
}

function getGoalLocation(robotLocation, heading) {
    let verticesAroundLocation = getVerticesAroundLocation(robotLocation); 
    let verticesAroundLocationRelative = shiftArray(verticesAroundLocation, heading, true);

    return verticesAroundLocationRelative[0]; // vertex in front of robot with front being relative to the heading of the robot.
}

function goalDetected(robot) {
    robotLib.robots.forEach((currentRobot) => {
        currentRobot.goalDetected = true;
        if(currentRobot.model === "PHYSICAL") {
            communication.publishMessage(`robots/toRobot/${currentRobot.robotID}/drive`, 0);
        } else {
            communication.publishMessage(`robots/toRobot/${currentRobot.robotID}/stop`, "");
            communication.publishMessage(`robots/toRobot/${currentRobot.robotID}/goalDetected`, "true");
        }
    });

    let goalLocation = getGoalLocation(robot.position, robot.heading);
    console.log(`SIMULATION COMPLETED\nGoal found at node number: ${goalLocation}`);
}

function registerRobot(topicArray) {
    if(robotLib.robotCounter >= robotLib.NUM_OF_ROBOTS) return;
    newRobot = robotLib.addRobot(`robot${robotLib.robotCounter++}`);
    topicArray[1] = "toRobot";
    communication.publishMessage(topicArray.join("/"), newRobot.robotID);
    console.log(`Published id: ${newRobot.robotID}`);
}

function calulateHeadingBetweenVertices(currentPosition, destination) {
    if (currentPosition - algorithms.columns == destination) {
        return 0;
    } else if (currentPosition + 1 == destination) {     
        return 90;
    } else if (currentPosition + algorithms.columns == destination) {
        return 180; 
    } else if (currentPosition - 1 == destination) {
        return 270;
    }
}

function rotateOrDrive(robot) { 
    let destination = robot.path[robot.pathIndex];
    let correctHeading = calulateHeadingBetweenVertices(robot.position, destination);
    let correctionAngle = correctHeading > robot.heading ? correctHeading - robot.heading : 360 - (robot.heading - correctHeading);
    console.log(correctionAngle, robot.heading)
    if (correctionAngle > ANGLE_MARGIN && correctionAngle < (360 - ANGLE_MARGIN)) {
        communication.publishMessage(`robots/toRobot/${robot.robotID}/rotateDegrees`, `${correctionAngle}`);
    } else {
        communication.publishMessage(`robots/toRobot/${robot.robotID}/drive`, `${MOTOR_SPEED}`);
    }
}

function sendPathOrInstructionToRobot() {
    robotLib.robots.forEach((currentRobot) => {
        console.log(currentRobot);
        if(currentRobot.model === "VIRTUAL") communication.publishMessage(
            `robots/toRobot/${currentRobot.robotID}/path`, `${currentRobot.path}`);
        else {
            rotateOrDrive(currentRobot);
        }
    });
}

function begin(robot){
    let begin_flag = false;
    if((robotLib.robotCounter === robotLib.NUM_OF_ROBOTS) && 
    robotLib.robots[(robotLib.robots.length)-1].robotID === robot.robotID){
        begin_flag = true;
    }
    if(!begin_flag) return;

    //Link all robots with the camera
    linkToCamera()

    let numberOfTargetSets = Math.round((algorithms.rows / robotLib.robotCounter) * 2);
    algorithms.targetSetMatrix = algorithms.createTargetSets(numberOfTargetSets);
    let startVertexVector = getRobotPositions();
    let numberOfVertices = algorithms.rows * algorithms.columns;
    let pathAndAssignment = algorithms.getPathAndAssignment(algorithms.adjacencyMatrix, numberOfVertices, startVertexVector, algorithms.targetSetMatrix[robotLib.targetSetIndex]);
    let optimalAssignments = munkres(pathAndAssignment[1]);
    for (assignmentIndex in optimalAssignments) {
        currentOptimalAssingment = optimalAssignments[assignmentIndex];
        robotLib.robots[assignmentIndex].path = pathAndAssignment[0][currentOptimalAssingment[0]][currentOptimalAssingment[1]];
    }
    // if(robot.position == 32) return; // TESTT
    sendPathOrInstructionToRobot();
}

function linkToCamera(){
    robotLib.robots.forEach(function(robot) {
        topic = "robots/toCamera/" + robot + "/link"
        communication.publishMessage(topic, "")
    });
}

function position(robot) {
    //visitedLocations[robot.position] = true;
    console.log(robot)
    if (robot.pathIndex === robot.path.length && robot.path.length > 0 && robot.state !== "WAITING") {
        if (robot.goalDetected) {
            communication.publishMessage(`robots/toRobot/${robot.robotID}/end`, "")
            return;
        }

        robotLib.readyForNextTarget++;
        robot.state = "WAITING"
        if (robotLib.readyForNextTarget === robotLib.NUM_OF_ROBOTS) {
            robotLib.readyForNextTarget = 0;
            let targetSet;
            if (robotLib.targetSetIndex !== algorithms.targetSetMatrix.length) {
                robotLib.targetSetIndex++;
                targetSet = algorithms.targetSetMatrix[robotLib.targetSetIndex];
            } else {
                return;
            }

            updatePaths(algorithms.adjacencyMatrix, targetSet);
            sendPathOrInstructionToRobot();
            robotLib.robots.forEach((robot) => {
                robot.state = "NO_TASK";
            });
        }
        // console.log(visitedLocations);
        return;
    }
    // TO DO CHECK PARAMETER (IS THE TOPIC THE SAME)
    if (robot.model === "VIRTUAL") {
        robot.pathIndex++;
        return;
    }

    if (robot.position === robot.path[robot.pathIndex + 1] ) {
        robot.pathIndex++;
        communication.publishMessage(`robots/toRobot/${robot.robotID}/drive`, "0");   //test if necessery

        rotateOrDrive(robot);
    }

    //bot drove wrong or not all bots begin
    
}

module.exports = {robotDetected, getVerticesAroundLocation, shiftArray, getObstacleLocations, getRobotPositions, updatePaths, 
  removeObstaclesFromAdjacencyMatrix, getUpdatedAdjacencyMatrix, obstacleDetected, getGoalLocation, goalDetected, 
  registerRobot, rotateOrDrive, begin, position};