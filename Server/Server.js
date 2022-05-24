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

function inputHandler(topicArray, message){

  //Call correct function depending on the 3rd array element

  return;
}

function getCurrentPos(robotID){
  return;
}

function calculatePos(robotID){
  return;
}

function sendNewPosToAll(){
  robots.forEach(robot => {
    sendNewPos(robot.robotID)
  });
}

function sendNewPos(robotID, newPos){
  let newPosTopic = `robots/${robotID}/newPos`;
  client.publish(newPosTopic, newPos, {qos: 0, retain: true});
  console.log(`published ${newPos} to robot with ID: ${robotID}`);
}

//Subscribe to all robot topics
client.on('connect', () => {
  console.log('Connected');
  client.subscribe([topicRobot], () => {
    console.log(`Subscribe to topic '${topicRobot}'`);
  });
});

/*
 * MSG format:
 * data transfer robot to server: robots/toServer/<robotID>/<robotProperty>
 * data transfer server to robot: robots/toRobot/<robotID>/<robotProperty>
 * Registering robot:             robots/toServer/subscribe/<randomNumber>
 */

client.on("message", (topic, message) => {
  let splitTopic = topic.split('/');

  if(!(splitTopic[0] === "robots" && splitTopic[1] === "toServer" && splitTopic.length > 2)){
    return;
  }

  if(robots.some(robot => robot.robotID === splitTopic[2])){
    console.log(`topic is: ${splitTopic[1]}, robots: ${JSON.stringify(robots)}`);

    inputHandler(splitTopic, message);
  }

  else if(splitTopic[2] === "register" && splitTopic.length > 2){
    console.log("Registering a new robot...")
    registerRobot(splitTopic);
  }
});