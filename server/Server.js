const mqtt = require("mqtt");


var robots = [];

//mqtt connection setup
const host = "127.0.0.1";
const port = "8005";
const clientId = "mqtt_1"; //Server is client 1
const connectUrl = `mqtt://${host}:${port}`;

const client = mqtt.connect(connectUrl, {
  clientId,
  clean: true,
  connectTimeout: 2000,
  username: 'backend',
  password: 'backend',
  reconnectPeriod: 1000,
});

const topicRobot = 'robots/#';


// Classes
class Robot {
   constructor(robotID){
     this.robotID = robotID;
     this.pos = [];
     this.newPos = [];
     this.heading = [];
     this.obstacle = false;
   }
}

// Function definitions

function addRobot(robotID){
  robots.push(new Robot(robotID));
}

function removeRobot(robotID){
  robots = robots.filter(robot => robot.robotID !== robotID);
}

function inputHandler(topic, message){
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
 *  MSG format:
 *  data transfer between robot and server: robots/<robotID>/<robotProperty>
 *  registering robots at the server:       robots/register/<random number>
 */

client.on("message", (topic, message) => {
  let splitTopics = topic.split('/');

  if(!(splitTopics[0] === "robots" && splitTopics.length > 1)){
    return;
  }

  if(robots.some(robot => splitTopics[1] === robot.robotID)){
    console.log(`topic is: ${splitTopics[1]}, robots: ${JSON.stringify(robots)}`);

    inputHandler(topic, message);

  }
});