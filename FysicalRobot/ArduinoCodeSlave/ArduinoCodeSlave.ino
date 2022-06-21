//grotere LED lamp: 220 ohm
#include <Wire.h>
const int aruinoAdress = 8;

const int ldrPin = A0;
const int irPin = 2;
const int irPin2 = 3;
byte ldrValue = 0;
int ldrValueAVG = 0;
int ldrValueCNT = 0;
byte irValue = 0;

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
  if(!digitalRead(irPin) || !digitalRead(irPin2)) irValue = 1;
  else irValue = 0;
  if(ldrValue > 4)
  {
    ldrValueAVG += ldrValue;
    ldrValueCNT++;
  }
  delay(20);
}

void requestEvent()
{
  byte ldrSendValue = ldrValueAVG / ldrValueCNT;

//  Serial.print("ldrSendValue:");
//  Serial.print(ldrSendValue);
//  Serial.print("ldrValueAVG:");
//  Serial.print(ldrValueAVG);
//  Serial.print("ldrValueCNT:");
//  Serial.println(ldrValueCNT);
  
  Wire.write(ldrSendValue);
  Wire.write(irValue);
  Serial.print("ldr:");
  Serial.print(ldrSendValue);
  Serial.print("\t");
  Serial.print("ir:");
  Serial.println(irValue*100);
  
  ldrValueAVG = 0;
  ldrValueCNT = 0;
}
