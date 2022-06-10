var munkres = require('munkres-js');


function createFirstTargetSet(numberOfRobots, columns) {
    var firstTargetSet = [];
    var firstTarget = 0;

    if (numberOfRobots <= 0) {
      return firstTargetSet;
    }

    firstTargetSet[0] = firstTarget;

    for (let firstTargetSetIndex = 1; firstTargetSetIndex < numberOfRobots; firstTargetSetIndex++) {
      firstTarget = firstTargetSetIndex * columns;
      firstTargetSet[firstTargetSetIndex] = firstTarget;
    }

    return firstTargetSet;
};

function fillTargetSets(firstTargetSet, numberOfTargetSets, targetSetPattern) {
    var targetSets = makeEmpty2DArray(numberOfTargetSets);
    var target;

    for (let firstTargetSetIndex = 0; firstTargetSetIndex < firstTargetSet.length; firstTargetSetIndex++) {
        target = firstTargetSet[firstTargetSetIndex];
        targetSets[0].push(target);

        for (let targetSetIndex = 0; targetSetIndex < numberOfTargetSets - 1; targetSetIndex++) {
        target += targetSetPattern[targetSetIndex % targetSetPattern.length];
        targetSets[targetSetIndex + 1].push(target);
        }
    }

    return targetSets;
};

function createAdjacencyMatrix(numberOfVertices) {
    var rows, columns;
    var index, rightCel, leftCel, celUp, celDown;
    var adjacencyMatrix = new Array(numberOfVertices).fill(0).map(function () { return new Array(numberOfVertices).fill(0); });
    rows = columns = Math.sqrt(numberOfVertices);

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

function breadthFirstSearch(adjacencyMatrix, startVertex, numberOfVertices) {
    var previous = makeEmpty2DArray(numberOfVertices);
    var visited = new Array(numberOfVertices).fill(false);
    var queue = [startVertex];

    visited[startVertex] = true;
    
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
    if (startVertex == targetVertex) {
        return [];
    }

    var path = [targetVertex];
    var previousStep = previous[targetVertex][0];

    while (previousStep != startVertex) {
      path.push(previousStep);
      previousStep = previous[previousStep][0];
    }

    return path.reverse();
};

function getPathAndAssignment (adjacencyMatrix, numberOfRobots, numberOfVertices, startVertexVector, targetSet) {
  console.log(adjacencyMatrix);
  var startVertex, targetVertex;
  var numberOfTargets = targetSet.length;
  var costMatrix = makeEmpty2DArray(numberOfRobots);
  var shortestPathStartToTarget;
  var shortestPathMatrix = [];

  for (let robotIndex = 0; robotIndex < numberOfRobots; robotIndex++) {
    shortestPathMatrix.push([]);
    for (let targetIndex = 0; targetIndex < numberOfTargets; targetIndex++) {
      startVertex = startVertexVector[robotIndex];
      targetVertex = targetSet[targetIndex];
      shortestPathStartToTarget = traceRoute(breadthFirstSearch(adjacencyMatrix, startVertex, numberOfVertices), startVertex, targetVertex);
      shortestPathMatrix[robotIndex].push(shortestPathStartToTarget);
      costMatrix[robotIndex][targetIndex] = shortestPathStartToTarget.length;
    }
  }

  return [shortestPathMatrix, costMatrix];
};

// /* Example code for using functions above.
var numberOfVertices = 9;
var numberOfRobots = 3;
var rows, columns;
rows = columns = Math.sqrt(numberOfVertices);

// var startVertexVector = [1,4,7];
// var firstTargetSet = createFirstTargetSet(numberOfRobots, columns);
// var numberOfTargetSets = Math.round((rows / numberOfRobots) * 2);
// var targetSetPattern = [rows-1, numberOfRobots * rows, -(rows - 1), numberOfRobots * rows];
// var targetVertexMatrix = fillTargetSets(firstTargetSet, numberOfTargetSets, targetSetPattern);

// var pathAndAssignment = getPathAndAssignment(numberOfRobots, numberOfVertices, startVertexVector, targetVertexMatrix[0]);
// var optimalAssignment = munkres(pathAndAssignment[1]);

// console.log(targetVertexMatrix);
// console.log(optimalAssignment);
// console.log(pathAndAssignment[1]);
// console.log(pathAndAssignment[0]);
// */

module.exports = { createFirstTargetSet, fillTargetSets, createAdjacencyMatrix, makeEmpty2DArray, breadthFirstSearch, traceRoute, getPathAndAssignment };