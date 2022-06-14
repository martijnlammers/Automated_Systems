const communication = require("./Communication");
const robotLib = require("./Robot");
const algorithms = require("./Controller")

// CHECK VARIABLE AND FUNCTION NAMES
function getVerticesAroundLocation(location) {
    var vertexAboveLocation = location - algorithms.columns;
    var vertexBelowLocation = location + algorithms.columns;
    var vertexLeftOfLocation = location - 1;
    var vertexRightOfLocation = location + 1;
    
    return [vertexAboveLocation, vertexRightOfLocation, vertexBelowLocation, vertexLeftOfLocation];
}

function shiftArray(array, heading, shiftRelative) {
    var angleBetweenVertices = 90;
    var amountToShift = heading / angleBetweenVertices;

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
    var verticesAroundLocation = getVerticesAroundLocation(robotLocation);
    var verticesAroundLocationRelative = shiftArray(verticesAroundLocation, heading, true);
    var obstaclePositions = [];

    for (index in obstacleDetectedList) {
        if (obstacleDetectedList[index] == true) {
            obstaclePositions.push(verticesAroundLocationRelative[index]);
        }
    }

    return obstaclePositions;
}

function getRobotPositions() {
    var robotPositions = [];

    for (robotIndex in robotLib.robots) {
        startVertexVector.push(robotLib.robots[robotIndex].position);
    }

    return robotPositions;
}

function updatePaths(targetSet) {
    var startVertexVector = getRobotPositions();
    var pathAndAssingment = algorithms.getPathAndAssignment(algorithms.adjacencyMatrix, robotLib.robots.length, rows * columns, startVertexVector, targetSet);
    var shortestPathMatrixRow, shortestPathMatrixColumn;

    optimalAssignment = munkres(pathAndAssingment[1]);
    shortestPathMatrix = pathAndAssingment[0];
    
    for (robotIndex in robotLib.robots) {
        shortestPathMatrixRow = optimalAssignment[robotIndex][0];
        shortestPathMatrixColumn = optimalAssignment[robotIndex][1];
        robotLib.robots[robotIndex].path = shortestPathMatrix[shortestPathMatrixRow][shortestPathMatrixColumn];
    }
}

function removeObstaclesFromAdjacencyMatrix(adjacencyMatrix, obstacleLocation) {
    for (obstacleIndex in obstacleLocation) { 
        var obstaclePosition = obstacleLocation[obstacleIndex];
        adjacencyMatrix[obstaclePosition].fill(0);
        
        for (row in adjacencyMatrix) {
            adjacencyMatrix[row][obstaclePosition] = 0;
        }
    }

    return adjacencyMatrix;
}

function getUpdatedAdjacencyMatrix(obstacleLocation, robotPositions) {
    var adjacencyMatrixWithoutObstacles = removeObstaclesFromAdjacencyMatrix(algorithms.adjacencyMatrix, obstacleLocation);
    var adjacencyMatrixCopy = JSON.parse(JSON.stringify(adjacencyMatrixWithoutObstacles));
    adjacencyMatrixCopy = removeObstaclesFromAdjacencyMatrix(adjacencyMatrixCopy, robotPositions);  

    return adjacencyMatrixCopy;
}

function obstacleDetected(robot) {
    var targetSet = algorithms.targetSetMatrix[robot.targetSetIndex];
    var obstacleLocations = getObstacleLocations(robot.position);
    var robotPositions = getRobotPositions();

    var updatedAdjacencyMatrix = getUpdatedAdjacencyMatrix(obstacleLocations, robotPositions)
    updatePaths(updatedAdjacencyMatrix, targetSet);
}

function getGoalLocation(robotLocation, heading) {
    var verticesAroundLocation = getVerticesAroundLocation(robotLocation); 
    var verticesAroundLocationRelative = shiftArray(verticesAroundLocation, heading, true);

    return verticesAroundLocationRelative[0]; // vertex in front of robot with front being relative to the heading of the robot.
}

function goalDetected(robot) {
    var goalLocation = getGoalLocation(robot.position, robot.heading);
    var locationLeftOfGoal = goalLocation - 1;
    var locationRightOfGoal = goalLocation + 1;
    var locationAboveGoal = goalLocation - algorithms.columns;
    var locationBelowGoal = goalLocation + algorithms.columns;
    var targetSet = [locationLeftOfGoal, locationRightOfGoal, locationAboveGoal, locationBelowGoal];

    updatePaths(algorithms.adjacencyMatrix, targetSet);
}

function registerRobot(topicArray) {
    newRobot = robotLib.addRobot(`robot${robotLib.robotCounter++}`);
    topicArray[1] = "toRobot";
    console.log(topicArray);
    communication.publishMessage(topicArray.join("/"), newRobot.robotID);
    console.log(`Published id: ${newRobot.robotID}`);
}

function rotateAck(){
  console.log("rotate bot stopped");
}

function begin(){
    if(!(robotLib.robotCounter === 1)) return;
    robotLib.robots.forEach((robot)=>{
        
        communication.updateInformation(robot.robotID, robot.model);
    })

    console.log(robotLib.robots.pop());
    //1 Get robot position (pos?)
    //2 
    //2 Get new position for each robot -> Pathfinding algo?
    //3 Assign new positions to robots
    //4 If found, stop assigning positions
    //5 Direct other robots to target found location
}

module.exports = {getVerticesAroundLocation, shiftArray, getObstacleLocations, getRobotPositions, updatePaths, 
  removeObstaclesFromAdjacencyMatrix, getUpdatedAdjacencyMatrix, obstacleDetected, getGoalLocation, goalDetected, 
  registerRobot, rotateAck, begin};