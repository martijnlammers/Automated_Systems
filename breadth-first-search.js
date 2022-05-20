function createAdjacencyMatrix(numberOfVertices) {
    var rows, columns;
    var index, rightCel, leftCel, celUp, celDown;
    var adjacencyMatrix = new Array(numberOfVertices).fill(0).map(function () { return new Array(numberOfVertices).fill(0); });
    rows = columns = Math.sqrt(numberOfVertices);
    for (var currentRow = 0; currentRow < rows; currentRow++) {
        for (var currentColumn = 0; currentColumn < columns; currentColumn++) {
            index = currentRow * columns + currentColumn;
            leftCel = currentRow * columns + currentColumn - 1;
            rightCel = currentRow * columns + currentColumn + 1;
            celUp = (currentRow - 1) * columns + currentColumn;
            celDown = (currentRow + 1) * columns + currentColumn;
            if (currentRow == 0 && currentColumn == 0) { // TOP LEFT
                adjacencyMatrix[index][rightCel] = 1;
                adjacencyMatrix[index][celDown] = 1;
            }
            else if (currentRow == 0 && currentColumn != columns - 1) { // TOP CENTER
                adjacencyMatrix[index][rightCel] = 1;
                adjacencyMatrix[index][celDown] = 1;
                adjacencyMatrix[index][leftCel] = 1;
            }
            else if (currentRow == 0 && currentColumn == columns - 1) { // TOP RIGHT
                adjacencyMatrix[index][celDown] = 1;
                adjacencyMatrix[index][leftCel] = 1;
            }
            else if ((currentRow > 0 && currentRow < rows - 1) && currentColumn == 0) { // CENTER LEFT
                adjacencyMatrix[index][celUp] = 1;
                adjacencyMatrix[index][rightCel] = 1;
                adjacencyMatrix[index][celDown] = 1;
            }
            else if ((currentRow > 0 && currentRow < rows - 1) && (currentColumn > 0 && currentColumn < columns - 1)) { // CENTER CENTER
                adjacencyMatrix[index][celUp] = 1;
                adjacencyMatrix[index][rightCel] = 1;
                adjacencyMatrix[index][celDown] = 1;
                adjacencyMatrix[index][leftCel] = 1;
            }
            else if ((currentRow > 0 && currentRow < rows - 1) && currentColumn == columns - 1) { // CENTER RIGHT
                adjacencyMatrix[index][celUp] = 1;
                adjacencyMatrix[index][leftCel] = 1;
                adjacencyMatrix[index][celDown] = 1;
            }
            else if (currentRow == rows - 1 && currentColumn == 0) { // BOTTOM LEFT
                adjacencyMatrix[index][celUp] = 1;
                adjacencyMatrix[index][rightCel] = 1;
            }
            else if (currentRow == rows - 1 && (currentColumn > 0 && currentColumn < columns - 1)) { // BOTTOM CENTER
                adjacencyMatrix[index][celUp] = 1;
                adjacencyMatrix[index][rightCel] = 1;
                adjacencyMatrix[index][leftCel] = 1;
            }
            else if (currentRow == rows - 1 && currentColumn == columns - 1) { // BOTTOM RIGHT
                adjacencyMatrix[index][celUp] = 1;
                adjacencyMatrix[index][leftCel] = 1;
            }
        }
    }
    return adjacencyMatrix;
}
function makeEmptyArray(numberOfVertices) {
    var array = [];
    for (var index = 0; index < numberOfVertices; index++) {
        array.push([]);
    }
    return array;
}
;
function breadthFirstSearch(startVertex, numberOfVertices) {
    var adjacencyMatrix = createAdjacencyMatrix(numberOfVertices);
    var previous = makeEmptyArray(numberOfVertices);
    var visited = new Array(9).fill(false);
    var queue = [startVertex];
    visited[startVertex] = true;
    while (queue.length > 0) {
        var visiting = queue[0];
        queue.shift();
        for (var i = 0; i < numberOfVertices; i++) {
            if (adjacencyMatrix[visiting][i] == 1 && (!visited[i])) {
                queue.push(i);
                visited[i] = true;
                previous[i].push(visiting);
            }
        }
    }
    return previous;
}
;
function traceRoute(previous, startVertex, endVertex) {
    var path = [endVertex];
    var previousStep = previous[endVertex][0];
    while (previousStep != startVertex) {
        path.push(previousStep);
        previousStep = previous[previousStep][0];
    }
    return path.reverse();
}
var numberOfVertices = 4;
var startVertex = 0;
var endVertex = 2;
var shortestPathStartToEnd = traceRoute(breadthFirstSearch(startVertex, numberOfVertices), startVertex, endVertex);
console.log("Shortest path from:", startVertex, "To", endVertex, "is", shortestPathStartToEnd);
console.log("Lenght shortest path is:", shortestPathStartToEnd.length);
