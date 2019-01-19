#include <LobotServoController.h>

LobotServoController myse(Serial);

void setup() {
  pinMode(13,OUTPUT);
  Serial.begin(9600);
  while(!Serial);
  digitalWrite(13,HIGH);

  myse.runActionGroup(100,0);  //loop run No.100 action group
  delay(5000);
  myse.stopActionGroup(); //stop running action group
  delay(2000);
  myse.setActionGroupSpeed(100,200); //Set the running speed of No.100 action group at 200%
  delay(2000);
  myse.runActionGroup(100,5);  //run No.100 action group 5 times 
  delay(5000);
  myse.stopActionGroup(); //stop running action group
  delay(2000);
  myse.moveServo(0,1500,1000); //move No.0 Servo in 1000ms to 1500 position
  delay(2000);
  myse.moveServo(2,800,1000); //move No.2 servo in 1000ms to 800 position
  delay(2000);
  myse.moveServos(5,1000,0,1300,2,700,4,600,6,900,8,790); 
  //Control 5 servos, action time is 1000ms, move No.0 servo to 1300 position, move No.2 servo to 700 position, move No.4 servo to 600 position
  //Move No.6 servo to 900 position, move No.8 servo to 790 position
  delay(2000);

  LobotServo servos[2];   //an array of struct LobotServo
  servos[0].ID = 2;       //No.2 servo
  servos[0].Position = 1400;  //1400 position
  servos[1].ID = 4;       //No.4 servo
  servos[1].Position = 700;  //700 position
  myse.moveServos(servos,2,1000);  //control 2 servos, action time is 1000ms, ID and position are specified by the structure array "servos"
}

void loop() {
}
