//grotere LED lamp: 220 ohm
#include <Wire.h>
const int aruinoAdress = 8;

const int ldrPin = A0;
const int irPin = 2;
//const int irPin2 = 3;
byte ldrValue = 0;
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
  irValueDown = !digitalRead(irPin);
//  irValueFront = !digitalRead(irPin2);
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
  
  Wire.write(ldrSendValue);
  Wire.write(irValueDown);
//  Wire.write(irValueFront);
  Serial.print("ldr:");
  Serial.print(ldrSendValue);
  Serial.print("\t");
  Serial.print("irDown:");
  Serial.print(irValueDown*100);
//  Serial.print("\t");
//  Serial.print("irFront:");
//  Serial.println(irValueFront*100);
  
  ldrValueAVG = 0;
  ldrValueCNT = 0;
}
