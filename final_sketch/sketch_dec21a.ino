//#include <i2cdetect.h>

#include <AFMotor.h>

#include <Keyboard.h>

#include "Wire.h"

#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"
#include <NewPing.h>

/*#define leftMotorPWMPin   6
#define leftMotorDirPin   7
#define rightMotorPWMPin  5
#define rightMotorDirPin  4

#define TRIGGER_PIN 9
#define ECHO_PIN 8
#define MAX_DISTANCE 75
*/
#define Kp  2550
#define Kd  0.0
#define Ki  0
#define sampleTime  0.005
#define targetAngle 0

MPU6050 mpu;
//NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
AF_DCMotor leftmotor(3);
AF_DCMotor rightmotor(1);

int16_t accX, accZ, gyroY;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
volatile byte count=0;
int distanceCm;

void setMotors(int leftMotorSpeed, int rightMotorSpeed) {
 
  if(leftMotorSpeed > 0) {
   // analogWrite(leftMotorPWMPin, leftMotorSpeed);
   //leftMotorSpeed=map(leftMotorSpeed,0,255,50,115);
   leftmotor.run(FORWARD);
   leftmotor.setSpeed(leftMotorSpeed);
   // digitalWrite(leftMotorDirPin, LOW);
  }
  else if(leftMotorSpeed == 0)
  leftmotor.run(RELEASE);
  else {
    //analogWrite(leftMotorPWMPin, 255 + leftMotorSpeed);
   // leftMotorSpeed=map(leftMotorSpeed,-255,0,-115,50);
    leftmotor.run(BACKWARD);
    leftmotor.setSpeed(-leftMotorSpeed);
    //digitalWrite(leftMotorDirPin, HIGH);
  }
  if(rightMotorSpeed > 0) {
    //analogWrite(rightMotorPWMPin, rightMotorSpeed);
    //digitalWrite(rightMotorDirPin, LOW);
    rightmotor.run(BACKWARD);
   rightmotor.setSpeed(rightMotorSpeed);
  }
  else if(rightMotorSpeed == 0)
  rightmotor.run(RELEASE);
  else {
   // analogWrite(rightMotorPWMPin, 255 + rightMotorSpeed);
    //digitalWrite(rightMotorDirPin, HIGH);
    rightmotor.run(FORWARD);
   rightmotor.setSpeed(-rightMotorSpeed);
  }
}



void init_PID() {  
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B    
  // set compare match register to set sample time 5ms
  OCR1A = 9999;    
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}

void setup() {
  // set the motor control and PWM pins to output mode
 /* pinMode(leftMotorPWMPin, OUTPUT);
  pinMode(leftMotorDirPin, OUTPUT);
  pinMode(rightMotorPWMPin, OUTPUT);
  pinMode(rightMotorDirPin, OUTPUT);
  */
  // set the status LED to output mode 
  pinMode(13, OUTPUT);
  Serial.begin(115200);
  // initialize the MPU6050 and set offset values
  mpu.initialize();
  mpu.setYAccelOffset(1593);
  mpu.setZAccelOffset(963);
  mpu.setXGyroOffset(40);
  // initialize PID sampling loop
  init_PID();
  /*accX = mpu.getAccelerationX();
  accZ = mpu.getAccelerationZ();  
  gyroY = mpu.getRotationY();
  accAngle = atan2(accX, accZ)*RAD_TO_DEG;
  //gyroRate = map(gyroY, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate*sampleTime;  
  currentAngle = 0.9934*(prevAngle + gyroAngle) + 0.0066*(accAngle);
  targetAngle=currentAngle;*/
}

void loop() {
  // read acceleration and gyroscope values
  accX = mpu.getAccelerationX();
  accZ = mpu.getAccelerationZ();  
  gyroY = mpu.getRotationY();
  // set motor power after constraining it
  motorPower = constrain(motorPower, -255, 255);
  setMotors(motorPower, motorPower);
  // measure distance every 100 milliseconds
/*  if((count%20) == 0){
    distanceCm = sonar.ping_cm();
  }
  if((distanceCm < 20) && (distanceCm != 0)) {
    setMotors(-motorPower, motorPower);
  }*/
}
// The ISR will be called every 5 milliseconds
ISR(TIMER1_COMPA_vect)
{
  // calculate the angle of inclination
  accAngle = atan2(accX, accZ)*RAD_TO_DEG;
  gyroRate = map(gyroY, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate*sampleTime;  
  Serial.print("gyroAngle:");
  Serial.println(gyroAngle);
  currentAngle = 0.9934*(prevAngle + gyroAngle) + 0.0066*(accAngle);
  Serial.print("currentAngle:");
  Serial.println(currentAngle);
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;  
  errorSum = constrain(errorSum, -300, 300);
  //calculate output from P, I and D values
  motorPower = Kp*(error) + Ki*(errorSum)*sampleTime - Kd*(currentAngle-prevAngle)/sampleTime;
  prevAngle = currentAngle;
  // toggle the led on pin13 every second
  Serial.print("error sum");
  Serial.print(errorSum);
  Serial.print("motorPower");
  Serial.print(motorPower);
  //count++;
 /* if(count == 200)  {
    count = 0;
    digitalWrite(13, !digitalRead(13));
  }*/
}
