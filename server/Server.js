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
});