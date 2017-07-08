#include <Arduino.h>

int sensorPin = A7;

void setup ()
{
  // analogReference(INTERNAL1V1);
  Serial.begin(115200);
  pinMode(sensorPin, INPUT);
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);
}

void loop()
{
  int sensorValue = analogRead(sensorPin);
  Serial.println(sensorValue);
  delay(50);
}
