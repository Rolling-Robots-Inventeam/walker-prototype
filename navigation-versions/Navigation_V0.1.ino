//Navigator Bot

#include <Servo.h>

Servo lservo; // write >90 to go forward
Servo rservo; //write <90 to go forward


//13 is bottom board/s blue LED and top one's Green.
//12 is top board's red.
//11 - 8 is the body of 4 pinslots on top.
//7 - 3 is the body of 5 pinslots on top.
//2 is the button on top (there are two, forgot which)

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600); // opens serial port

  lservo.attach(9); //attach to servo itself our servo control object
  rservo.attach(11);
  
  pinMode(6, INPUT); //left infared bumper
  pinMode(7, INPUT); //right infared bumper

  pinMode(12, OUTPUT); //nice red led for debug use


  delay(1000);
}



//The infared sensors go low when something's in range
//so these functions invert that 
bool LeftBumper(){ return !digitalRead(6); }
bool RightBumper(){ return !digitalRead(7); }

void drive (int left, int right){
  lservo.write (92.8 + left);
  rservo.write (92.8 - right);
  }

//constant of initial
//the base speed at which we drive forward
int kI = 4;

//constant of bumper
//immediate motor adjustment when coming in range of a wall
int kB = 2;

//constant of tick
//the rate at which that adjustment grows per update loop spent in range of wall
float kT = 0.5;

//constant of steer
//strength of beacon heading's effect on the drive
int kS = 1;

float leftPower;
float rightPower;
int lTick = 0;
int rTick = 0;
int steer = 0; //assume positive goes right, negative goes left

void loop() {

  // power = baseval + 2-part anti-wall adjustment + beacon steering
  leftPower = kI + (LeftBumper() * kB) + (lTick * kT) - (steer * kS); 
  rightPower =  kI + (RightBumper() * kB) + (rTick * kT) - (steer * kS); 

  //tick makes it so that the longer the wall is within 10cm, the faster away it drives
  lTick = (lTick + LeftBumper()) * LeftBumper(); //resets whenever bool is 0
  rTick = (rTick + RightBumper()) * RightBumper(); //++ whenever bool is 1
  //speed of increase directly correlated to update rate (adjust with kT)
  
  drive(leftPower, rightPower);

  //an led to display whenever we're in range of a wall
  if (LeftBumper() || RightBumper())
  { digitalWrite(12, HIGH); }
  else { digitalWrite (12, LOW); }

  delay(250);  

}
