#include <LobotServoController.h>

#define rxPin 10
#define txPin 11

SoftwareSerial mySerial(rxPin, txPin);
LobotServoController myse(mySerial);

void setup() {
  pinMode(13,OUTPUT);
  mySerial.begin(9600);
  Serial.begin(9600);
}
void loop() {
  digitalWrite(13, HIGH);
  myse.runActionGroup(1,1);
  if(myse.waitForStopping(10000))  //Waiting for the action group is completed, or be stopped or timeout. The unit of the parameter(timeout) is milliscond. finish or stop, you will get a true. timeout, you will get a false
  {
    Serial.println("FINISH");
    digitalWrite(13, LOW);
  }
  else
    Serial.println("NOT FINISH");
  delay(1000);
}
