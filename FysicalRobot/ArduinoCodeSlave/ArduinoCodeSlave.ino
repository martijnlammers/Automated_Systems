//grotere LED lamp: 220 ohm
#include <Wire.h>
const int aruinoAdress = 8;

const int ldrPin = A0;
const int irPin = 2;
//const int irPin2 = 3;
int ldrValue = 0;
int ldrValueAVG = 0;
int ldrValueCNT = 0;
byte irValueDown = 0;
//byte irValueFront = 0;

void setup()
{
  Serial.begin(115200);
  pinMode (irPin, INPUT);
  Wire.begin(aruinoAdress);
  Wire.onRequest(requestEvent);
}

void loop()
{
  ldrValue = analogRead(ldrPin);
  Serial.print(" ");
  Serial.print(ldrValue);
  irValueDown = !digitalRead(irPin);
//  irValueFront = !digitalRead(irPin2);
  if(ldrValue > 4)
  {
    ldrValueAVG += ldrValue;
//    Serial.print(" ");
//    Serial.print(ldrValueAVG);
    ldrValueCNT++;
  }
  delay(20);
}

void requestEvent()
{
  bool goalDetected = false;
  int ldrSendValue = ldrValueAVG / ldrValueCNT;
  if(ldrSendValue > 450) 
  {
    goalDetected = true;
  }
  
  Wire.write(goalDetected);
  Wire.write(irValueDown);
//  Wire.write(irValueFront);
  Serial.print("\n goal:");
  Serial.print(goalDetected);
  Serial.print("\t");
  Serial.print("irDown:");
  Serial.println(irValueDown*100);
//  Serial.print("\t");
//  Serial.print("irFront:");
//  Serial.println(irValueFront*100);
  
  ldrValueAVG = 0;
  ldrValueCNT = 0;
}
