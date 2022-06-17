import { DijkstraCalculator } from 'dijkstra-calculator';

const graph = new DijkstraCalculator();
for(let i = 0; i < 100; i++){
    graph.addVertex(i);
}

for(let i = 0; i < 10; i++){
    for(let j = 0; j < 10; j++){
        console.log(i, j+10)
        graph.addEdge(i, j+10, 4);
    }
}

