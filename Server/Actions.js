const communication = require("./Communication");
const robotLib = require("./Robot");
const algorithms = require("./Controller");
const munkres = require("munkres-js");

const ANGLE_MARGIN = 3;
const MOTOR_SPEED = 90;
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

function getUpdatedAdjacencyMatrix(obstacleLocation, robotPositions) {
    // var adjacencyMatrixCopy = JSON.parse(JSON.stringify(algorithms.adjacencyMatrix));
    
    for (obstacleIndex in obstacleLocation) { 
        let obstaclePosition = obstacleLocation[obstacleIndex]; 
        // console.log("obstaclePosition", obstaclePosition);
        if (robotPositions.includes(obstaclePosition)) continue
        else removeObstaclesFromAdjacencyMatrix(algorithms.adjacencyMatrix, obstaclePosition);
    }

    return algorithms.adjacencyMatrix;
}

function getTargetSet() {
    if (robotLib.targetSetIndex !== algorithms.targetSetMatrix.length) {
        // console.log("MATRIX");
        return algorithms.targetSetMatrix[robotLib.targetSetIndex];
    } else {
        // console.log("UNVISITED");
        let targetSet = [];
        let targetCounter = 0;
        while(targetCounter < robotLib.NUM_OF_ROBOTS) {
            let unvisitedLocation = visitedLocations.findIndex(location => location == false);
            if (unvisitedLocation == -1) return false;
            visitedLocations[unvisitedLocation] = true;
            targetSet.push(unvisitedLocation);
            targetCounter++;
        }
        return targetSet;
    }
}

function obstacleDetected(robot) {
    // console.log("obstacleDetected", robot.robotID);
    robotLib.robots.forEach((currentRobot) => {
        // console.log("ObstacleDetected", currentRobot.robotID);
        if(currentRobot.model === "PHYSICAL") {
            communication.publishMessage(`robots/toRobot/${currentRobot.robotID}/drive`, 0);
        } else {
            communication.publishMessage(`robots/toRobot/${currentRobot.robotID}/stop`, "");
        }
    });
    // console.log("=================================");
    // console.log(robotLib.targetSetIndex);
    let obstacleLocations = getObstacleLocations(robot.position, robot.heading, robot.obstacleDetected);
    let robotPositions = getRobotPositions();
    obstacleLocations.forEach((location) => {visitedLocations[location] = true;});
    robotPositions.forEach((location) => {visitedLocations[location] = true;});
    // console.log("obstacleLocations:", obstacleLocations);

    let updatedAdjacencyMatrix = getUpdatedAdjacencyMatrix(obstacleLocations, robotPositions);
    let targetSet = getTargetSet();
    if (!targetSet) return;
    updatePaths(updatedAdjacencyMatrix, targetSet);

    sendPathOrInstructionToRobot();
}

function getGoalLocation(robotLocation, heading) {
    let verticesAroundLocation = getVerticesAroundLocation(robotLocation); 
    let verticesAroundLocationRelative = shiftArray(verticesAroundLocation, heading, true);

    return verticesAroundLocationRelative[0]; // vertex in front of robot with front being relative to the heading of the robot.
}

function goalDetected(robot) {
    console.log("goalDetected", robot.robotID);
    robotLib.robots.forEach((currentRobot) => {
        console.log("robotID", currentRobot.robotID);
        console.log("position", currentRobot.position);
        currentRobot.goalDetected = true;
        if(currentRobot.model === "PHYSICAL") {
            communication.publishMessage(`robots/toRobot/${currentRobot.robotID}/drive`, 0);
        } else {
            communication.publishMessage(`robots/toRobot/${currentRobot.robotID}/stop`, "");
            communication.publishMessage(`robots/toRobot/${currentRobot.robotID}/goalDetected`, "true");
        }
    });

    let goalLocation = getGoalLocation(robot.position, robot.heading);
    console.log("goalLocation", goalLocation);
    let locationLeftOfGoal = goalLocation - 1;
    let locationRightOfGoal = goalLocation + 1;
    let locationAboveGoal = goalLocation - algorithms.columns;
    let locationBelowGoal = goalLocation + algorithms.columns;
    let targetSet = [locationLeftOfGoal, locationRightOfGoal, locationAboveGoal, locationBelowGoal];
    // let targetSet = [locationBelowGoal, locationLeftOfGoal];

    removeObstaclesFromAdjacencyMatrix(algorithms.adjacencyMatrix, goalLocation);
    updatePaths(algorithms.adjacencyMatrix, targetSet);
    sendPathOrInstructionToRobot();

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
        // console.log("path:", currentRobot.path);

        if(currentRobot.model === "VIRTUAL") communication.publishMessage(`robots/toRobot/${currentRobot.robotID}/path`, `${currentRobot.path}`);
        // else rotateOrDrive(currentRobot);
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
        // console.log(robotLib.robots[assignmentIndex].path);
    }
    // console.log(3);
    sendPathOrInstructionToRobot();
}

function position(robot) {
    console.log("position", robot.robotID);
    console.log(robot.path);

    visitedLocations[robot.position] = true;

    // console.log(robot.path.length, robot.pathIndex);
    if (robot.pathIndex === robot.path.length && robot.path.length > 0) {
        if (robot.goalDetected) {
            communication.publishMessage(`robots/toRobot/${robot.robotID}/end`, "")
            console.log("robotID done", robot.robotID);
            return;
        }
        robotLib.readyForNextTarget++;
        console.log("readyForNextTarget", robotLib.readyForNextTarget);
        if (robotLib.readyForNextTarget === robotLib.NUM_OF_ROBOTS) {
            robotLib.readyForNextTarget = 0;
            if (robotLib.targetSetIndex !== algorithms.targetSetMatrix.length) robotLib.targetSetIndex++;

            let targetSet = getTargetSet();
            if (!targetSet) return;
            updatePaths(algorithms.adjacencyMatrix, targetSet);
            
            sendPathOrInstructionToRobot();
        }
        
        return;
    }
    // TO DO CHECK PARAMETER (IS THE TOPIC THE SAME)
    if (robot.model === "VIRTUAL") {
        // console.log("test1");
        robot.pathIndex++;
        return;
    }

    if (robot.position === robot.path[robot.pathIndex + 1] ) {
        // console.log("test2");
        robot.pathIndex++;
        communication.publishMessage(`robots/toRobot/${robot.robotID}/drive`, 0);   //test if necessery

        sendInstructionToRobot();
    }

    //bot drove wrong
    
}

module.exports = {getVerticesAroundLocation, shiftArray, getObstacleLocations, getRobotPositions, updatePaths, 
  removeObstaclesFromAdjacencyMatrix, getUpdatedAdjacencyMatrix, obstacleDetected, getGoalLocation, goalDetected, 
  registerRobot, rotateOrDrive, begin, position};