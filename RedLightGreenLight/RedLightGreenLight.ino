//Analog distance sensor
#include <MedianFilter.h>
#include <SharpDistSensor.h>

#include <Wire.h>
#include "SparkFun_VL53L1X.h"

#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

// Pin configuration
const int sensorPin = A0; // Analog input pin

SFEVL53L1X distanceSensor;
SharpDistSensor sensor(sensorPin);

int ledPinA = D0;
int ledPinD = D3;
int buttonPin = D5;
int analogValue1 = 0;
int analogValue2 = 0;
int digitalValue1 = 0;
int digitalValue2 = 0;
int buttonState = 0;
int start = 0;
int buzzerPin = D4;  

void setup() {
  pinMode(buttonPin,INPUT);
  pinMode(ledPinA, OUTPUT);
  pinMode(ledPinD, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  Wire.begin();

  Serial.begin(115200);
  //Serial.println("VL53L1X Qwiic Test");

  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor online!");
}

void loop(void) {

  delay(500);
  for (int k=0;k<10;k++)
  {
    delay(100);
    digitalWrite(buzzerPin, HIGH);
    delay(100);
    digitalWrite(buzzerPin, LOW);
      
  }

  buttonState = digitalRead(buttonPin);
  if( buttonState==HIGH)
    start=1;
  while( start==1)
  {
    digitalWrite(buzzerPin, HIGH);
    delay(1000);
    digitalWrite(buzzerPin, LOW);
    for (int k=0;k<3;k++)
    {
      delay(100);
      digitalWrite(buzzerPin, HIGH);
      delay(100);
      digitalWrite(buzzerPin, LOW);
      
    }

    for(int i = 0; i < 10; i++)
    {

      
      analogValue1=0;
      digitalValue1=0;
      analogValue2=0;
      digitalValue2=0;


      Serial.print("\n-------------------------------------\n ");
      for (int j=0;j<10;j++)
      {
        analogValue1 += sensor.getDist();
        
        delay(10);
      }
      analogValue1/=10;
      Serial.print("A1: ");
      Serial.print(analogValue1);

      distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
      while (!distanceSensor.checkForDataReady())
      {
        delay(1);
      }

      digitalValue1 = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
      distanceSensor.clearInterrupt();
      distanceSensor.stopRanging();
    
      Serial.print("\t\tD1: ");
      Serial.print(digitalValue1);

      Serial.println();
      delay(200);
      
      
      for (int j=0;j<10;j++)
      {
        analogValue2 += sensor.getDist();
        delay(10);
      }
      analogValue2/=10;
      Serial.print("A2: ");
      Serial.print(analogValue2);
      Serial.println(" cm");

      distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
      while (!distanceSensor.checkForDataReady())
      {
        delay(1);
      }

      digitalValue2 = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
      distanceSensor.clearInterrupt();
      distanceSensor.stopRanging();
      Serial.print("\t\tD2: ");
      Serial.print(digitalValue2);

      if(analogValue1 - analogValue2 > 20 || analogValue2 - analogValue1 > 20 || digitalValue2 - digitalValue1 > 20 || digitalValue1 - digitalValue2 > 20)
    {
        digitalWrite(buzzerPin, HIGH);
        if(analogValue1 - analogValue2 > 20 || analogValue2 - analogValue1 > 20)
        {
          Serial.print("\nmove analog ");
          digitalWrite(ledPinA, HIGH);
        }
        if(digitalValue2 - digitalValue1 > 20 || digitalValue1 - digitalValue2 > 20)
        {
          Serial.print("\nmove digital ");
          digitalWrite(ledPinD, HIGH); 
        }
        delay(100);
        digitalWrite(ledPinA, LOW);
        digitalWrite(ledPinD, LOW);
        digitalWrite(buzzerPin, LOW);
      }
      Serial.println();

      buttonState = digitalRead(buttonPin);
      if( buttonState==HIGH)
        start=0;
      
    }
  }
}