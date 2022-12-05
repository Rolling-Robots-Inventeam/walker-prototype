// ======= DEFINITIONS =======

// Ultrasonic definitions
#define echoPin 12 // attach pin D12 Arduino to pin Echo of HC-SR04
#define trigPin 13 //attach pin D13 Arduino to pin Trig of HC-SR04

// Infrared definitions
#define sensor A3 // Sharp IR GP2Y0A41SK0F (4-30cm, analog)

// ======= VARIABLES =======

// Ultrasonic variables
long duration; // variable for the duration of sound wave travel
int distanceUltrasonic; // variable for the distance measurement

// Infrared variables
int distanceIR;

// Pressure variables
int fsrAnalogPin = 1; // FSR is connected to analog 1
int pressureReading;      // the analog reading from the FSR resistor divider

void setupUltrasonic() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
}

void loopUltrasonic() {
  // delay(1000);
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distanceUltrasonic = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  // Serial.print("Distance: ");
  // Serial.print(distanceUltrasonic);
  // Serial.println(" cm");
}

void loopInfrared() {
  float volts = analogRead(sensor)*0.0048828125;  // value from sensor * (5/1024)
  distanceIR = 13*pow(volts, -1); // worked out from datasheet graph
  
  // Serial.println(distanceIR);   // print the distance
}

void loopPressure() {
  pressureReading = analogRead(fsrAnalogPin);
  // Serial.print("Analog reading = ");
  // Serial.println(pressureReading);
}

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing");

  setupUltrasonic();
}

void loop() {
  delay(1000);
  loopUltrasonic();
  loopInfrared();
  loopPressure();

  Serial.print("Ultrasonic: ");
  Serial.println(distanceUltrasonic);

  Serial.print("Infrared: ");
  Serial.println(distanceIR);

  Serial.print("Pressure: ");
  Serial.println(pressureReading);
  
  Serial.println("\n===============\n");
}
