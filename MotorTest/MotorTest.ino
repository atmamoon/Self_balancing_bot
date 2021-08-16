// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!

//#include <AFMotor.h>
//#include <Servo.h>
#define leftMotorPWMPin   7
#define leftMotorDirPin   6
#define rightMotorPWMPin  4
#define rightMotorDirPin  5

//AF_DCMotor leftmotor(1);
//AF_DCMotor rightmotor(4);
//Servo servo1;
//#define angle;


void setMotors(int leftMotorSpeed, int rightMotorSpeed) {
  if(leftMotorSpeed > 0) {
    analogWrite(leftMotorPWMPin, leftMotorSpeed);
  // leftmotor.run(FORWARD);
   //leftmotor.setSpeed(leftMotorSpeed);
   digitalWrite(leftMotorDirPin, LOW);
  }
  //else if(leftMotorSpeed == 0)
  //leftmotor.run(RELEASE);
  else {
  analogWrite(leftMotorPWMPin,  leftMotorSpeed);
//    leftmotor.run(BACKWARD);
  //  leftmotor.setSpeed(-leftMotorSpeed);
    digitalWrite(leftMotorDirPin, HIGH);
  }
 // servo1.write(angle);
 /* if(rightMotorSpeed >= 0) {
    analogWrite(rightMotorPWMPin, rightMotorSpeed);
    digitalWrite(rightMotorDirPin, LOW);
    //rightmotor.run(FORWARD);
   //rightmotor.setSpeed(rightMotorSpeed);
  }
  //else if(rightMotorSpeed == 0)
  //rightmotor.run(RELEASE);
  else {
    analogWrite(rightMotorPWMPin, 255 + rightMotorSpeed);
    digitalWrite(rightMotorDirPin, HIGH);
    //rightmotor.run(BACKWARD);
   //rightmotor.setSpeed(-rightMotorSpeed);
  }
  */
}



void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Motor test!");
  //servo1.attach(9);
  // turn on motor
  //leftmotor.setSpeed(200);
 
 // leftmotor.run(RELEASE);
}

void loop() {
/* if (i>=180)
    i=0;
  for(int i=0;i<180;i+=5)
  {
    servo1.write(i);
    
  }*/
  setMotors(-250,-255);
  delay(100);
}
