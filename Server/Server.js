const mqtt = require("mqtt");


var robots = [];
var robotCounter = 0;

//mqtt connection setup
const host = "127.0.0.1";
const port = "8005";
const clientId = "mqtt_1"; //Server is client 1
const connectUrl = `mqtt://${host}:${port}`;

const client = mqtt.connect(connectUrl, {
  clientId,
  clean: true,
  connectTimeout: 4000,
  username: 'backend',
  password: 'backend',
  reconnectPeriod: 1000,
});

const topicRobot = 'robots/toServer/#';


class Robot {
  constructor(robotID){
    this.robotID = robotID;
    this.position = [];
    this.newPosition = [];
    this.heading = 0;
    this.obstacleDetected = [0, 0];
    this.targetDetected = false;
  }
}

// Function definitions
function registerRobot(topicArray){
  newRobot = addRobot(`robot${robotCounter++}`);
  topicArray[1] = "toRobot";
  client.publish(topicArray.join("/"), newRobot.robotID);
  console.log(`Published id: ${newRobot.robotID}`);
}

function addRobot(robotID){
  newRobot = new Robot(robotID);
  robots.push(newRobot);
  return newRobot;
}

function removeRobot(robotID){
  robots = robots.filter(robot => robot.robotID !== robotID);
}

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
    }
  }

  else{
    //Send error code to robot
    console.log("Error: invalid topic!");
  }
}

function getCurrentPos(robotID){
  return;
}

function calculatePos(robotID){
  return;
}

function sendNewPosToAll(){
  robots.forEach(robot => {
    sendNewPos(robot)
  });
}

function sendNewPos(robot){
  let newPosTopic = `robots/toRobot/${robot.robotID}/newPos`;
  client.publish(newPosTopic, robot.newPos, {qos: 0, retain: false});
  console.log(`published ${robot.newPos} to robot with ID: ${robot.robotID}`);
}

//Subscribe to all robot topics
client.on('connect', () => {
  console.log('Connected');
  client.subscribe([topicRobot], () => {
    console.log(`Subscribe to topic '${topicRobot}'`);
  });
});

client.on("message", (topic, message) => {
  let splitTopic = topic.split('/');
  message = message.toString();

  if(!(splitTopic[0] === "robots" && splitTopic[1] === "toServer" && splitTopic.length > 2)){
    return;
  }
  
  if(robots.some(r => r.robotID === splitTopic[2])){
    let robot = robots.filter(r => r.robotID === splitTopic[2]).pop();

    if(robot != undefined){
      inputHandler(robot, splitTopic, message);
    }
  }

  else if(splitTopic[2] === "register" && splitTopic.length > 2){
    console.log("Registering a new robot...")
    registerRobot(splitTopic);
  }
});