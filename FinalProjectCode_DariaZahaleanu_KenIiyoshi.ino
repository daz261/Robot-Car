#include "Wire.h"
#include "sensorbar.h"

//line follower address
const uint8_t SX1509_ADDRESS = 0x3E;  // SX1509 I2C address (00)

//color sensor 1 mapped to digital pins 0 ~ 4.
int S0_right = 0;
int S1_right = 1;
int S2_right = 2;
int S3_right = 3;
int out_right = 4;
//color sensor 2 mapped to digital pins 5 ~ 9.
int S0_left = 5;
int S1_left = 6;
int S2_left = 7;
int S3_left = 8;
int out_left = 9;

//distance sensor
int echo = A0;
int trigger = A1;

// H-bridge 1
//PWM can only be analog and 11, 10, 9, 6, 5, 3
int rightPWM = 10; //purple
int rightDIR = 12; //yellow

//H-bridge 2
int leftPWM = 11; //purple
int leftDIR = 13; //yellow


// Ken - Lower speeds will not be enough to move the car smoothly.
int speed = 50;
int leftSpeed = 20;
int rightSpeed = 20;
int greenFrequency_left;
int greenFrequency_right;

int dutyCycle = 200;

//Define the states that the decision making machines uses:
#define IDLE_STATE 0
#define READ_LINE 1
#define GO_FORWARD 2
#define GO_BACKWARD 3
#define GO_LEFT 4
#define GO_RIGHT 5

uint8_t state;
uint8_t nextState;
uint8_t rawValue = 0;
int duration;
int distance = 0;
volatile uint16_t critDistance=9276; //10cm
int greenColor_right, greenColor_left = 0;

SensorBar mySensorBar(SX1509_ADDRESS);

void setup() {
  // sets the pins as outputs:
  pinMode(rightPWM, OUTPUT);
  pinMode(rightDIR, OUTPUT);
  pinMode(leftPWM, OUTPUT);
  pinMode(leftDIR, OUTPUT);

  // sets the left color sensor pins as outputs:
  pinMode(S0_left, OUTPUT);
  pinMode(S1_left, OUTPUT);
  pinMode(S2_left, OUTPUT);
  pinMode(S3_left, OUTPUT);

  pinMode(echo, OUTPUT);
  pinMode(trigger, OUTPUT);

  // sets the right color sensor pins as outputs:
  pinMode(S0_right, OUTPUT);
  pinMode(S1_right, OUTPUT);
  pinMode(S2_right, OUTPUT);
  pinMode(S3_right, OUTPUT);
  
  pinMode(out_left, INPUT);
  pinMode(out_right, INPUT);

   // Setting frequency scaling to 20%
  digitalWrite(S0_left,HIGH);
  digitalWrite(S1_left,LOW);
  digitalWrite(S0_right,HIGH);
  digitalWrite(S1_right,LOW);

  // begin
  Serial.begin(9600);  // start serial for output
  Serial.println("Program started.");
  Serial.println();

  // the IR will only be turned on during reads.
  mySensorBar.setBarStrobe();

  //  With inversion cleared, the sensor is looking for a dark line on light background
  mySensorBar.clearInvertBits();
  uint8_t returnStatus = mySensorBar.begin();
  
  if(returnStatus)
  {
    Serial.println("sx1509 IC communication OK");
  }
  else
  {
    Serial.println("sx1509 IC communication FAILED!");
  }
  Serial.println();
}

void loop() {
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);

  duration = pulseIn(echo, HIGH);
  distance= duration*0.034/2;
  Serial.print("Distance: ");
  Serial.println(distance);
  
  
  Serial.print("Position (-127 to 127): ");
  Serial.println(mySensorBar.getPosition());

  Serial.println("xPosition: ");
  for( int i = 7; i >= 0; i-- )
  {
    Serial.print((rawValue >> i) & 0x01);
  }
  Serial.println("b");
  
  Serial.print("Density, bits detected (of 8): ");
  Serial.println(mySensorBar.getDensity());
  rawValue = mySensorBar.getRaw();


  switch (state) {
  case IDLE_STATE:
    stopMoving();
    nextState = READ_LINE;
    break;
  case READ_LINE:
    if( mySensorBar.getDensity() <= 8 )
    {
      nextState = GO_FORWARD;
      //check the distance in cm
      //if <2, it means that it detects an obstacle
      if (distance < 2){
          bwd();
        }
      // if line is detected on the left side of the robot.
      if( mySensorBar.getPosition() <   -50 )
      {
        nextState = GO_LEFT;
        if (distance < 2){
          bwd();
      }
      // if line is detected on the right side of the robot.
      if( mySensorBar.getPosition() > 50 )
      {
        nextState = GO_RIGHT;
        if (distance < 2){
          bwd();
      }
    }
    else{
     nextState = IDLE_STATE;
    }
    break;
  case GO_FORWARD:
    defaultFct();
   // fwd();
    nextState = READ_LINE;
    break;
  case GO_BACKWARD:
    bwd();
    nextState = READ_LINE;
    break;
  case GO_LEFT:
    left();
    nextState = READ_LINE;
    break;
  case GO_RIGHT:
    right();
    nextState = READ_LINE;
    break;
  // if state = something we did not expect:
  default:
    stopRight();
    stopLeft();    // Stops both motors
    break;
  }
  state = nextState;

colorSensor_left();
colorSensor_right();

  }
  }
}
//read the value of the IR sensors on the line follower array
int getBar(int i) {
  return (rawValue >> i) & 0x01;
}

void defaultFct() {
  //case 1: intersection
  if (getBar(0) == 1 && getBar(1) == 1 && getBar(2) == 1 && getBar(3) == 1 && 
  getBar(4) == 1 && getBar(5) == 1 && getBar(6) == 1 && getBar(7) == 1) { //intersection
      fwd();
      if (getBar(0) == 1 && getBar(1) == 1 && getBar(2) == 1){
        //check if green square is detected
          if(greenColor_right >=30){
          sharpRight(); //sharp right
          }
          else if (greenColor_right >=30 && greenColor_left >=30 ){ //both squares detected, turn around
            turnAround();
           }
          else{ //no square detected
            fwd();
            }
     }
     else if (getBar(5) == 1 && getBar(6) == 1 && getBar(7) == 1){
      if(greenColor_left >=30){
         sharpLeft(); //sharp left
      }
       else if (greenColor_right >=30 && greenColor_left >=30 ){ //both squares detected, turn around
            turnAround();
           }
       else{ //no square detected
            fwd();
            }
  }
     else if (getBar(3) == 0 && getBar(4) == 1 ){
       if(greenColor_left >=30){
          left(); //turn left
       }
       else if (greenColor_right >=30 && greenColor_left >=30 ){ //both squares detected, turn around
            turnAround();
           }
       else{ //no square detected
            fwd();
            }
      }
      else{
          if(greenColor_right >=30){
          right(); //turn right
          }
          else if (greenColor_right >=30 && greenColor_left >=30 ){ //both squares detected, turn around
            turnAround();
           }
          else{ //no square detected
            fwd();
            }
        }
    }
    //case 2: stops
  else if (getBar(0) == 0 && getBar(1) == 0 && getBar(2) == 0 && getBar(3) == 0 && 
  getBar(4) == 0 && getBar(5) == 0 && getBar(6) == 0 && getBar(7) == 0){ //stop
      stopMoving();  
  }
  //case 3: forward
  else if (getBar(3) == 0 && getBar(4) == 0){
    fwd();
  }
   //case 4: intersection and then steering left or right
  else if (getBar(3) == 1 && getBar(4) == 1 ){ //intersection and then turn right
     if (getBar(0) == 1 && getBar(1) == 1 && getBar(2) == 1){
          if(greenColor_right >=30){
          sharpRight(); //sharp right
          }
     }
     else if (getBar(5) == 1 && getBar(6) == 1 && getBar(7) == 1){
       if(greenColor_left >=30){
         sharpLeft(); //sharp left
       }
  }
     else if (getBar(3) == 0 && getBar(4) == 1 ){
        if(greenColor_left >=30){
          left(); //turn left
        }
      }
      else{
        if(greenColor_right >=30){
        right(); //turn right
          }
        }
  }
  }

//forward
void fwd(){
  //  Serial.println("Moving Forward");
    digitalWrite(rightDIR, HIGH);
    analogWrite(rightPWM, speed); 
    digitalWrite(leftDIR, HIGH); 
    analogWrite(leftPWM, speed);
 }

//backward
void bwd(){
 // Serial.println("Moving Backward");
  digitalWrite(rightDIR, LOW);
  analogWrite(rightPWM, speed); 
  digitalWrite(leftDIR, LOW); 
  analogWrite(leftPWM, speed);
 }

//stop right motor
void stopRight(){
  digitalWrite(rightDIR, LOW);
  analogWrite(rightPWM, 0);
}

//stop left motor
void stopLeft(){
  digitalWrite(leftDIR, LOW);
  analogWrite(leftPWM, 0);
}

//stop both motors
void stopMoving(){
  Serial.println("Stop Moving");
  digitalWrite(rightDIR, LOW);
  analogWrite(rightPWM, 0);
  digitalWrite(leftDIR, LOW);
  analogWrite(leftPWM, 0);
}

//move left
void left(){
 Serial.println("Moving Left");
 digitalWrite(rightDIR, LOW);
 analogWrite(rightPWM, leftSpeed); 
 digitalWrite(leftDIR, HIGH); 
 analogWrite(leftPWM, leftSpeed);
}

//move right
void right(){
  Serial.println("Moving Right");
  digitalWrite(rightDIR, HIGH);
  analogWrite(rightPWM, rightSpeed); 
  digitalWrite(leftDIR, LOW); 
  analogWrite(leftPWM, rightSpeed);
}

void sharpRight(){
  rightSpeed = 10;
   Serial.println("Moving Right");
  digitalWrite(rightDIR, HIGH);
  analogWrite(rightPWM, rightSpeed); 
  digitalWrite(leftDIR, LOW); 
  analogWrite(leftPWM, rightSpeed);
}

void sharpLeft(){
  leftSpeed = 10;
 Serial.println("Moving Left");
 digitalWrite(rightDIR, LOW);
 analogWrite(rightPWM, leftSpeed); 
 digitalWrite(leftDIR, HIGH); 
 analogWrite(leftPWM, leftSpeed);
}

void turn(int deg) {
  speed = 60;
  if (deg > 0) {
    deg = map(deg, 0, 180, 0, -1600);
  }
  else {
    deg = map(deg, -180, 0, 1600, 0);
  }
  if (deg > 0) {
    digitalWrite(rightDIR, HIGH);
    analogWrite(rightPWM, speed);
    stopRight();
  }
  else if (deg < 0) {
    deg = -deg;
    digitalWrite(leftDIR, HIGH);
    analogWrite(leftPWM, speed);
    stopLeft();
  }
  speed = 20;
}

//turn around function
void turnAround(){
 turn(360); 
  }
  
//left color sensor 
// min & max greenFrequency_left: 50ish and 280 ish.
void colorSensor_left(){
  // Setting GREEN (G) filtered photodiodes to be read
  digitalWrite(S2_left,HIGH);
  digitalWrite(S3_left,HIGH);
  
  greenFrequency_left = pulseIn(out_left, LOW);
  //cutoff = 30, under artificial light conditions
  greenColor_left = map(greenFrequency_left, 50, 280, 255, 0);
 //Serial.print(greenColor);
 
  Serial.print(" G_left = ");
  Serial.print(greenFrequency_left);
  delay(100);
  }

//right color sensors
// min & max greenFrequency_right: 5 ish and 8 ish.
void colorSensor_right(){
  // Setting GREEN (G) filtered photodiodes to be read
  digitalWrite(S2_right,HIGH);
  digitalWrite(S3_right,HIGH);
 
  greenFrequency_right = pulseIn(out_right, LOW);
  //cutoff = 30, under artificial light conditions
  greenColor_right = map(greenFrequency_right, 50, 280, 255, 0);
  //Serial.print(greenColor);
  
  Serial.print(" G_right = ");
  Serial.print(greenFrequency_right);
  delay(100);
  }

 
