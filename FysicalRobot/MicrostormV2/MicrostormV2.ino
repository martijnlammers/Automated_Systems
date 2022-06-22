#include "EspMQTTClient.h"
#include<Wire.h>

//MQTT
EspMQTTClient client(
  "Tesla IoT",
  "fsL6HgjN",
//  "RidderBerg_Guest",
//  "ditisgeenwachtwoord",
  "145.24.222.37",     // MQTT Broker server ip
  "robots",
  "robots",
  "MicroStormClient1", // Client name that uniquely identify your device
  8005
);

//defining pins
const int PWM1 = 13;
#define AIN1 26
#define AIN2 27
const int PWM2 = 25;
#define BIN1 32
#define BIN2 33
const int arduinoAdress = 8; //I2C
unsigned long lastReadTime;
int nextReadTime = 100;
bool goalDetected = false;
bool obstacleDetected = false;
bool robotDetected = false;
bool goalDetectedSend = false;
bool obstacleDetectedSend = false;
bool robotDetectedSend = false;

int motorChannel1 = 0;
int motorChannel2 = 1;
int motorFreq = 5000;
int motorRes = 8;

int motorSpeedStnd = 80;
int cornerDelay = 1000; //SCH>600<1200  Th<900
float delayForOneDegree = cornerDelay / 90;
String robotId;

unsigned long startStepTime;
int totalStepTime;
String lastStep = "";
bool stepDone = true;

void setup()
{
  Serial.begin(115200);
  
  pinMode(PWM1, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  ledcSetup(motorChannel1, motorFreq, motorRes);
  ledcSetup(motorChannel2, motorFreq, motorRes);
  ledcAttachPin(PWM1, motorChannel1);
  ledcAttachPin(PWM2, motorChannel2);

  client.enableDebuggingMessages(); //MQTT

  Wire.begin(); //I2C

  while(!client.isMqttConnected()) client.loop();
}

void loop()
{
  client.loop();  //MQTT

  if(millis()-lastReadTime > nextReadTime) readI2C();  //I2C

  if(goalDetected && goalDetectedSend == false)
  {
    client.publish("robots/toServer/" +robotId+ "/goalDetected", "");
    goalDetectedSend = true;
    setMotorSpeed(0);
  }
  if(obstacleDetected && obstacleDetectedSend == false)
  {
    client.publish("robots/toServer/" +robotId+ "/obstacleDetected", "1,0,0,0");  //[1,0,0,0] for sending that 
    obstacleDetectedSend = true;
    setMotorSpeed(0);
  }
  if(robotDetected && robotDetectedSend == false)
  {
    client.publish("robots/toServer/" +robotId+ "/robotDetected", "1,0,0,0");  //[1,0,0,0] for sending that 
    robotDetectedSend = true;
    setMotorSpeed(0);
  }
  
  if((millis()-startStepTime > totalStepTime) && stepDone == false)
  {
    Serial.print("millis: ");
    Serial.println(millis());
    Serial.println("step stopped");
    stepDone = true;
    setMotorSpeed(0);
    //send MQTT stopped
    client.publish("robots/toServer/<robotId>/rotateAck", "true");
  }
}

void onConnectionEstablished() //MQTT Connection
{
  client.subscribe("robots/toRobot/register/239803456798023740958273", succesfullRegistered);
  client.publish("robots/toServer/register/239803456798023740958273", "");
}

void succesfullRegistered(const String& payload) //MQTT Connection
{
  Serial.print("robotId: ");
  Serial.println(payload);
  robotId = payload;
  client.subscribe("robots/toRobot/" +robotId+ "/rotateDegrees", rotateDegrees);
  client.subscribe("robots/toRobot/" +robotId+ "/drive", drive);
  client.unsubscribe("robots/toRobot/register/239803456798023740958273");
  client.publish("robots/toServer/" +robotId+ "/begin", "");
}

void readI2C()
{
  Wire.requestFrom(arduinoAdress,2);

  while(Wire.available())
  {
    byte ldrValue = Wire.read();
    if(ldrValue > 250) goalDetected = true;
    if(goalDetected && ldrValue < 250) goalDetected = false;
    
//    Serial.print("ldr:");
//    Serial.print(ldrValue);
//    Serial.print("\t");
//    Serial.print("ir:");
    
    byte irValueDown = Wire.read();   // is 1 if no obstacle
    byte irValueFront = Wire.read();  // is 0 if no robot
    if(!irValueDown) obstacleDetected = true;
    if(irValueFront) robotDetected = true;
    if(obstacleDetected && irValueDown) obstacleDetected = false;
    if(robotDetected && !irValueFront) robotDetected = false;
    
//    Serial.println(irValue*100);

    lastReadTime = millis();
  }
}

void drive(const String& message)
{
  int motorSpeed = message.toInt();
  Serial.print("driving started, motorspeed: ");
  Serial.println(motorSpeed);
  setMotorSpeed(motorSpeed);
}

void rotateDegrees(const String& message)
{
  int rotateDegrees = message.toInt();
  Serial.print("rotating started, rotateDegrees: ");
  Serial.println(rotateDegrees);
  if(rotateDegrees < 180) driveRight(motorSpeedStnd, (int)(rotateDegrees*delayForOneDegree));
  else driveLeft(motorSpeedStnd, (int)(360-rotateDegrees)*delayForOneDegree);
}

void setMotorSpeed(int motorSpeed)    //motorSpeed=0 for STOP
{
  digitalWrite(AIN1, HIGH); //Motor A Rotate Clockwise
  digitalWrite(AIN2, LOW);
  ledcWrite(motorChannel1, motorSpeed);
  digitalWrite(BIN1, HIGH); //Motor B Rotate Clockwise
  digitalWrite(BIN2, LOW);
  ledcWrite(motorChannel2, motorSpeed);
}

void driveRight(int motorSpeed, int driveTime)
{
  digitalWrite(AIN1, HIGH); //Motor A Rotate Clockwise
  digitalWrite(AIN2, LOW);
  ledcWrite(motorChannel1, motorSpeed);
  digitalWrite(BIN1, LOW); //Motor B Rotate Counterclockwise
  digitalWrite(BIN2, HIGH);
  ledcWrite(motorChannel2, motorSpeed);
  
  startStepTime = millis();
  totalStepTime = driveTime;
  Serial.print("driveTimeRight: ");
  Serial.println(driveTime);
  Serial.println(startStepTime);
  Serial.println(totalStepTime);
  stepDone = false;
}

void driveLeft(int motorSpeed, int driveTime)
{
  digitalWrite(AIN1, LOW); //Motor A Rotate Counterclockwise
  digitalWrite(AIN2, HIGH);
  ledcWrite(motorChannel1, motorSpeed);
  digitalWrite(BIN1, HIGH); //Motor B Rotate Clockwise
  digitalWrite(BIN2, LOW);
  ledcWrite(motorChannel2, motorSpeed);
  
  startStepTime = millis();
  totalStepTime = driveTime;
  Serial.print("driveTimeLeft: ");
  Serial.println(driveTime);
  Serial.println(startStepTime);
  Serial.println(totalStepTime);
  stepDone = false;
}
