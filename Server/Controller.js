var munkres = require('munkres-js');
var robotLib = require("./Robot");

const rows = 10;
const columns = 10;

function createFirstTargetSet() {
    var firstTargetSet = [];
    var firstTarget = 0;

    if (robotLib.robotCounter <= 0) {
      return firstTargetSet;
    }

    firstTargetSet[0] = firstTarget;

    for (let firstTargetSetIndex = 1; firstTargetSetIndex < robotLib.robotCounter; firstTargetSetIndex++) {
      firstTarget = firstTargetSetIndex * columns;
      firstTargetSet[firstTargetSetIndex] = firstTarget;
    }

    return firstTargetSet;
};

function createTargetSets(numberOfTargetSets) {
    targetSetMatrix = makeEmpty2DArray(numberOfTargetSets)
    var targetSetPattern = [rows-1, robotLib.robotCounter * rows, -(rows - 1), robotLib.robotCounter * rows];
    var firstTargetSet = createFirstTargetSet();
    var targetSetMatrix = makeEmpty2DArray(numberOfTargetSets);
    var target;

    for (let firstTargetSetIndex = 0; firstTargetSetIndex < firstTargetSet.length; firstTargetSetIndex++) {
        target = firstTargetSet[firstTargetSetIndex];
        targetSetMatrix[0].push(target);

        for (let targetSetIndex = 0; targetSetIndex < numberOfTargetSets - 1; targetSetIndex++) {
        target += targetSetPattern[targetSetIndex % targetSetPattern.length];
        targetSetMatrix[targetSetIndex + 1].push(target);
        }
    }

    return targetSetMatrix;
}; 

function createAdjacencyMatrix() {
    var numberOfVertices = rows * columns;
    var index, rightCel, leftCel, celUp, celDown;
    var adjacencyMatrix = new Array(numberOfVertices).fill(0).map(function () { return new Array(numberOfVertices).fill(0); });
    for (var currentRow = 0; currentRow < rows; currentRow++) {
        for (var currentColumn = 0; currentColumn < columns; currentColumn++) {
            index = currentRow * columns + currentColumn;
            leftCel = index - 1;
            rightCel = index + 1;
            celUp = index - columns;
            celDown = index + columns;
            
            if (currentRow == 0) { 
                if (currentColumn == 0) { // TOP LEFT
                    adjacencyMatrix[index][rightCel] = 1;
                    adjacencyMatrix[index][celDown] = 1;
                }
                else if (currentColumn != columns - 1) { // TOP CENTER
                    adjacencyMatrix[index][rightCel] = 1;
                    adjacencyMatrix[index][celDown] = 1;
                    adjacencyMatrix[index][leftCel] = 1;
                }
                else if (currentColumn == columns - 1) { // TOP RIGHT
                    adjacencyMatrix[index][celDown] = 1;
                    adjacencyMatrix[index][leftCel] = 1;
                }
            }
            else if (currentRow > 0 && currentRow < rows - 1) {
                if (currentColumn == 0) { // CENTER LEFT
                    adjacencyMatrix[index][celUp] = 1;
                    adjacencyMatrix[index][rightCel] = 1;
                    adjacencyMatrix[index][celDown] = 1;
                }
                else if (currentColumn > 0 && currentColumn < columns - 1) { // CENTER CENTER
                    adjacencyMatrix[index][celUp] = 1;
                    adjacencyMatrix[index][rightCel] = 1;
                    adjacencyMatrix[index][celDown] = 1;
                    adjacencyMatrix[index][leftCel] = 1;
                }
                else if (currentColumn == columns - 1) { // CENTER RIGHT
                    adjacencyMatrix[index][celUp] = 1;
                    adjacencyMatrix[index][leftCel] = 1;
                    adjacencyMatrix[index][celDown] = 1;
                }
            }
            else if (currentRow == rows - 1) {
                if (currentColumn == 0) { // BOTTOM LEFT
                    adjacencyMatrix[index][celUp] = 1;
                    adjacencyMatrix[index][rightCel] = 1;
                }
                else if (currentColumn > 0 && currentColumn < columns - 1) { // BOTTOM CENTER
                    adjacencyMatrix[index][celUp] = 1;
                    adjacencyMatrix[index][rightCel] = 1;
                    adjacencyMatrix[index][leftCel] = 1;
                }
                else if (currentColumn == columns - 1) { // BOTTOM RIGHT
                    adjacencyMatrix[index][celUp] = 1;
                    adjacencyMatrix[index][leftCel] = 1;
                }
            }
            
        }
    }

    return adjacencyMatrix;
};

function makeEmpty2DArray(numberOfRows) {
    var array = [];
    for (let index = 0; index < numberOfRows; index++) {
        array.push([]);
    }
    return array;
};

function breadthFirstSearch(adjacencyMatrix, robotPositions, startVertex, numberOfVertices) {
    var previous = makeEmpty2DArray(numberOfVertices);
    var visited = new Array(numberOfVertices).fill(false);
    var queue = [startVertex];
    visited[startVertex] = true;
    robotPositions.forEach((robotPosition) => {
        visited[robotPosition] = true;
    });

    while (queue.length > 0) {
        var visiting = queue[0];
        queue.shift();

        for (let i = 0; i < numberOfVertices; i++) {
            if (adjacencyMatrix[visiting][i] == 1 && (!visited[i])) {
                queue.push(i);
                visited[i] = true;
                previous[i].push(visiting);
            }
        }
    }
    return previous;
};

function traceRoute(previous, startVertex, targetVertex) {
    console.log("traceRoute begin");
    console.log("startVertex", startVertex);
    console.log("endVertex", targetVertex);
    console.log("previous", previous);
    var path = [targetVertex];
    if (startVertex == targetVertex) return path;

    var previousStep = previous[targetVertex][0];
    while (previousStep != startVertex) {
        path.push(previousStep);
        console.log(previousStep);
        previousStep = previous[previousStep][0];
    }
    // console.log("traceRoute end");
    return path.reverse();
};

function getPathAndAssignment (adjacencyMatrix, numberOfVertices, startVertexVector, targetSet) {
  console.log("pathAndAssignment begin");
  var startVertex, targetVertex;
  var numberOfTargets = targetSet.length;
  var costMatrix = makeEmpty2DArray(robotLib.robotCounter);
  var shortestPathStartToTarget;
  var shortestPathMatrix = [];

  for (let robotIndex = 0; robotIndex < robotLib.robotCounter; robotIndex++) {
    shortestPathMatrix.push([]);
    for (let targetIndex = 0; targetIndex < numberOfTargets; targetIndex++) {
      startVertex = startVertexVector[robotIndex];
      targetVertex = targetSet[targetIndex];
      console.log("indices (robot) (target)",robotIndex, targetIndex);
      console.log("startVertexVector",startVertexVector);
      console.log("startVertex",startVertex);
      if (startVertexVector.includes(targetVertex) && targetVertex !== startVertex) {
        shortestPathMatrix[robotIndex].push([]);
        costMatrix[robotIndex][targetIndex] = Number.POSITIVE_INFINITY;
        continue;
      } 
      let previous = breadthFirstSearch(adjacencyMatrix, startVertexVector, startVertex, numberOfVertices);
      console.log(previous);
      shortestPathStartToTarget = traceRoute(previous, startVertex, targetVertex);
      shortestPathMatrix[robotIndex].push(shortestPathStartToTarget);
      costMatrix[robotIndex][targetIndex] = shortestPathStartToTarget.length;
    }
  }
  console.log("shortestPaths", shortestPathMatrix);
  console.log("costMatrix", costMatrix);
  console.log("pathAndAssignment end");
  return [shortestPathMatrix, costMatrix];
};

var adjacencyMatrix = createAdjacencyMatrix(rows, columns);
var targetSetMatrix;

// console.log(adjacencyMatrix);
/* Example code for using functions above.
var numberOfVertices = rows * columns;

var startVertexVector = [1,4,7];
var numberOfTargetSets = Math.round((rows / 3) * 2);
targetSetMatrix = createTargetSets(numberOfTargetSets)

var pathAndAssignment = getPathAndAssignment(adjacencyMatrix, numberOfVertices, startVertexVector, targetSetMatrix[0]);
var optimalAssignment = munkres(pathAndAssignment[1]);

console.log(targetSetMatrix);
console.log(optimalAssignment);
console.log(pathAndAssignment[1]);
console.log(pathAndAssignment[0]);
*/

module.exports = { createFirstTargetSet, createTargetSets, createAdjacencyMatrix, makeEmpty2DArray, breadthFirstSearch, traceRoute,
     getPathAndAssignment, adjacencyMatrix, targetSetMatrix, rows, columns };
