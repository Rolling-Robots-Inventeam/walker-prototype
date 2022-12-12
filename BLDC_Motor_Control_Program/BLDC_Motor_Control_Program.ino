#include <Servo.h>

const int rmPin = 9;//Arduino pin for motor
const int lmPin = 10;//Arduino pin for motor
const int RX_PIN = A0;//Arduino pin for joystick x-axis
const int RY_PIN = A1;//Arduino pin for joystick y-axis

double P1;//power value ranging from 1000-2000 sent to motor
double P2;//power value ranging from 1000-2000 sent to motor

Servo driveR;//creates a servo object under name "driveR"
Servo driveL;//creates a servo object under name "driveR"

void setup() {
  driveR.attach(rmPin, 1000, 2000);//tells the program where driveR is connected
  driveL.attach(lmPin, 1000, 2000);//tells the program where driveR is connected
  pinMode(rmPin, OUTPUT);//sets the pin to output mode
  pinMode(lmPin, OUTPUT);//sets the pin to output mode
  Serial.begin(9600);//starts a serial output
}

void loop() {
  set_Speed();//calls the set_Speed function
  drive();//calles drive function
}

void drive() {
  driveR.writeMicroseconds(P1);//tells the motor to move according to P1
  driveL.writeMicroseconds(P2);//tells the motor to move according to P1
  Serial.println(P1);//prints out a counting variable to check if the code is running
  Serial.print(P2);
}

void set_Speed() {
  P1 = analogRead(RX_PIN);//sets P1 equal to the inputted value
  P1 = map(P1, 0, 1023, 1000, 2000); //remaps power values from 0-180 to 1000-2000

  P2 = analogRead(RY_PIN);//sets P1 equal to the inputted value
  P2 = map(P2, 0, 1023, 1000, 2000); //remaps power values from 0-180 to 1000-2000
}
