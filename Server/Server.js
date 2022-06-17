const robotLib = require("./Robot");
const communication = require("./Communication");
const actions = require("./Actions");
let restart = 0;

var commands = {
  "rotateAck": actions.rotateOrDrive,
  "obstacleDetected": actions.obstacleDetected,
  "goalDetected": actions.goalDetected,
  "begin": actions.begin,
  "position": actions.position
};

function inputHandler(robot, topicArray, message){
  if(Object.getOwnPropertyNames(robot).some(property => property == topicArray[3])){
    switch(typeof(robot[topicArray[3]])){
      case "number":
        robot[topicArray[3]] = parseInt(message);
        break;
      case "object":
        if(Array.isArray(robot[topicArray[3]])){
          
          //Convert numbers in a string (ex: "1,2,3") to int array
          robot[topicArray[3]] = message.split(",").map(Number);
        }
        break;
      case "boolean":
        robot[topicArray[3]] = (message.toLowerCase() === "true");
        break;
      case "string":
        robot[topicArray[3]] = message;
        break;
    }
  }
}

//Subscribe to all robot topics
communication.client.on('connect', () => {
  console.log('Connected');
  communication.client.subscribe([communication.topicRobot], (err, granted) => {

    //Exit on error
    if(granted.length === 0 || err !== null){
      console.log("An error has occured, check if the server is already running!")
      process.kill(process.pid, 'SIGTERM');
    }
    
    console.log(`Subscribe to topic '${communication.topicRobot}'`);
  });
});

communication.client.on("message", (topic, message) => {
  let splitTopic = topic.split('/');
  message = message.toString();

  if(!(splitTopic[0] === "robots" && splitTopic[1] === "toServer" && splitTopic.length > 2)) return;

  if(robotLib.robots.some(r => r.robotID === splitTopic[2])){
    var robot = robotLib.robots.filter(r => r.robotID === splitTopic[2]).pop();
    inputHandler(robot, splitTopic, message);
  } else if(splitTopic[2] === "register" && splitTopic.length > 2){
    actions.registerRobot(splitTopic);
    return;
  }

  action = commands[splitTopic[3]];
  if(action === undefined) return;
  if(action.length === 0) action();
  else if(robot !== null || robot !== undefined) action(robot);
});


