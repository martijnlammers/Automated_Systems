const communication = require("./Communication");
const robotLib = require("./Robot");
const algorithms = require("./Controller");
const munkres = require("munkres-js");

const ANGLE_MARGIN = 3;
const MOTOR_SPEED = 90;

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
    shortestPathMatrix = pathAndAssingment[0];
    
    for (robotIndex in robotLib.robots) {
        shortestPathMatrixRow = optimalAssignment[robotIndex][0];
        shortestPathMatrixColumn = optimalAssignment[robotIndex][1];
        robotLib.robots[robotIndex].path = shortestPathMatrix[shortestPathMatrixRow][shortestPathMatrixColumn];
    }
}

function removeObstaclesFromAdjacencyMatrix(adjacencyMatrix, obstaclePosition) {
    adjacencyMatrix[obstaclePosition].fill(0);
    
    for (row in adjacencyMatrix) {
        adjacencyMatrix[row][obstaclePosition] = 0;
    }
}

function getUpdatedAdjacencyMatrix(obstacleLocation, robotPositions) {
    var adjacencyMatrixCopy = JSON.parse(JSON.stringify(algorithms.adjacencyMatrix));
    
    for (obstacleIndex in obstacleLocation) { 
        let obstaclePosition = obstacleLocation[obstacleIndex]; 
        console.log("obstaclePosition", obstaclePosition);
        if (robotPositions.includes(obstaclePosition)) removeObstaclesFromAdjacencyMatrix(adjacencyMatrixCopy, obstaclePosition);
        else {
            removeObstaclesFromAdjacencyMatrix(algorithms.adjacencyMatrix, obstaclePosition);
            removeObstaclesFromAdjacencyMatrix(adjacencyMatrixCopy, obstaclePosition);
        }
    }

    return adjacencyMatrixCopy;
}

function obstacleDetected(robot) {
    robotLib.robots.forEach((currentRobot) => {
        if(currentRobot.model === "PHYSICAL") {
            communication.publishMessage(`robots/toRobot/${currentRobot.robotID}/drive`, 0);
        } else {
            communication.publishMessage(`robots/toRobot/${currentRobot.robotID}/stop`, "");
        }
    });

    let targetSet = algorithms.targetSetMatrix[robotLib.targetSetIndex];
    let obstacleLocations = getObstacleLocations(robot.position, robot.heading, robot.obstacleDetected);
    let robotPositions = getRobotPositions();
    let updatedAdjacencyMatrix = getUpdatedAdjacencyMatrix(obstacleLocations, robotPositions);
    console.log("=================================");
    updatePaths(updatedAdjacencyMatrix, targetSet);

    sendPathOrInstructionToRobot();
}

function getGoalLocation(robotLocation, heading) {
    let verticesAroundLocation = getVerticesAroundLocation(robotLocation); 
    let verticesAroundLocationRelative = shiftArray(verticesAroundLocation, heading, true);

    return verticesAroundLocationRelative[0]; // vertex in front of robot with front being relative to the heading of the robot.
}

function goalDetected(robot) {
    let goalLocation = getGoalLocation(robot.position, robot.heading);
    let locationLeftOfGoal = goalLocation - 1;
    let locationRightOfGoal = goalLocation + 1;
    let locationAboveGoal = goalLocation - algorithms.columns;
    let locationBelowGoal = goalLocation + algorithms.columns;
    let targetSet = [locationLeftOfGoal, locationRightOfGoal, locationAboveGoal, locationBelowGoal];

    updatePaths(algorithms.adjacencyMatrix, targetSet);
}

function registerRobot(topicArray) {
    if(robotLib.robotCounter >= robotLib.NUM_OF_ROBOTS) return;
    newRobot = robotLib.addRobot(`robot${robotLib.robotCounter++}`);
    topicArray[1] = "toRobot";
    console.log(topicArray);
    communication.publishMessage(topicArray.join("/"), newRobot.robotID);
    console.log(`Published id: ${newRobot.robotID}`);
}

function calulateHeadingBetweenVertices(currentPosition, destination) {
    if (currentPosition - columns == destination) {
        return 0;
    } else if (currentPosition + 1 == destination) {     
        return 90;
    } else if (currentPosition + columns == destination) {
        return 180; 
    } else if (currentPosition - 1 == destination) {
        return 270;
    }
}

function rotateOrDrive(robot) { 
    let destination = robot.path[robot.pathIndex + 1];
    let correctHeading = calulateHeadingBetweenVertices(robot.position, destination);

    let correctionAngle = correctHeading > robot.heading ? correctHeading - robot.heading : 360 - (robot.heading - correctHeading);
    if (correctionAngle > ANGLE_MARGIN && correctionAngle < (360 - ANGLE_MARGIN)) {
        communication.publishMessage(`robots/toRobot/${robot.robotID}/rotateDegrees`, correctionAngle);
    } else {
        communication.publishMessage(`robots/toRobot/${robot.robotID}/drive`, MOTOR_SPEED);
    }
}

function sendPathOrInstructionToRobot() {
    robotLib.robots.forEach((currentRobot) => {
        if (currentRobot.path.length === 0) return;

        if(currentRobot.model === "VIRTUAL") communication.publishMessage(`robots/toRobot/${currentRobot.robotID}/setPath`, `${currentRobot.path}`);
        else rotateOrDrive(currentRobot);
    });
}

function begin(robot){
    let begin_flag = false;
    if((robotLib.robotCounter === robotLib.NUM_OF_ROBOTS) && 
    robotLib.robots[(robotLib.robots.length)-1].robotID === robot.robotID){
        begin_flag = true;
    }
    if(!begin_flag) return;
    // communication.publishMessage(`robots/toRobot/${robot.robotID}/setPos`, `${0}`);
    // communication.publishMessage(`robots/toRobot/${robot.robotID}/rotateDegrees`, `${270}`);
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

    sendPathOrInstructionToRobot();
}

function position(robot) {
    if (robot.pathIndex === robot.path.length && robot.path.length > 0) {
        robotLib.readyForNextTarget++;
        if (robotLib.readyForNextTarget === robotLib.NUM_OF_ROBOTS) {  //last robot sends pos before others?
            robotLib.readyForNextTarget = 0;
            robotLib.targetSetIndex++;

            let targetSet = algorithms.targetSetMatrix[robotLib.targetSetIndex];
            updatePaths(algorithms.adjacencyMatrix, targetSet);
            
            sendPathOrInstructionToRobot();
        }
        
        return;
    }
    // TO DO CHECK PARAMETER (IS THE TOPIC THE SAME)
    if (robot.model === "VIRTUAL") {
        robot.pathIndex++;
    }

    if (robot.position === robot.path[robot.pathIndex + 1] ) {
        robot.pathIndex++;
        communication.publishMessage(`robots/toRobot/${robot.robotID}/drive`, 0);   //test if necessery

        sendInstructionToRobot();
    }

    //bot drove wrong
    
}

module.exports = {getVerticesAroundLocation, shiftArray, getObstacleLocations, getRobotPositions, updatePaths, 
  removeObstaclesFromAdjacencyMatrix, getUpdatedAdjacencyMatrix, obstacleDetected, getGoalLocation, goalDetected, 
  registerRobot, rotateOrDrive, begin, position};