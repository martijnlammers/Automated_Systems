const mqtt = require("mqtt");
const host = "127.0.0.1", port = "8005";
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

function publishMessage(topic, message){
  client.publish(topic, message);
}

module.exports = { host, port, clientId, connectUrl, client, topicRobot, publishMessage};