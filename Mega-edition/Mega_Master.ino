

/*
 *  PROGRAM INFORMATION
 * 
 *  This is the comprehensive arudino sketch for the 2022-2023 Rolling Robots InvenTeam's Walker Project.
 *  It features, predominantly, the walker's algorithm for navigating rooms to get to its user.
 *  It also has basic logic for automatic braking.
 * 
 *  This sketch is designed to be as liquid as possible with hardware. All inputs and outputs should be customizable and easily integrated.
 */


/*
  *  MEGA MASTER CHIP
  *  This chip handles almost every process.
  *   - Sensor data, including handbrake and various distance sensors
  *   - All Bluetooth tracking data
  *   - Hosts navigation algorithm
  *   - Motor controlling via vesc serials
  */



// -----
#include <VescUart.h>
#include <SoftwareSerial.h>
#include <stdlib.h>

// -----

bool debugresponse = true;

//------------------------

// Pins

// Analog
int LeftHandbrake = 14;   // A0
int RightHandbrake = 15;  // A1
int LeftInfared = 16;     // A2
int RightInfared = 17;    // A3
//int ANALOG4 = 18; // A4
//int ANALOG5 = 19; // A5

// Digital

int UltrasonicTrig = 2;
int UltrasonicEcho = 22;

int MicroCtrlIMU_rx = 10;
int MicroCtrlIMU_tx = 11;
int MicroCtrlBLE_rx = 12;
int MicroCtrlBLE_tx = 13;

//-------------------------


// ------- Motor Start -------

VescUart vescML;
VescUart vescMR;


float getMotorRPM(VescUart vesc) {
  if (vesc.getVescValues()) {
    float motor_rpm = vesc.data.rpm;

    Serial.print("RPM: "); Serial.println(motor_rpm);
    return motor_rpm;
  }
  else
  {
    float motor_rpm = 9999;
    
    Serial.println("Failed to get data!");
    return motor_rpm;
  }
}

// ------- Motor End -------


// ------- Sensor Start -------

// Ultrasonic variableas
int distanceUltrasonic; // variable for the distance measurement

void loopUltrasonic() {
  
  // Clears the trigPin condition
    digitalWrite(UltrasonicTrig, LOW);
    delayMicroseconds(2);
    
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    digitalWrite(UltrasonicTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(UltrasonicTrig, LOW);
    
  // Reads the echoPin, returns the sound wave travel time in microseconds
    long duration = pulseIn(UltrasonicEcho, HIGH); // variable for the duration of sound wave travel
    
  // Calculating the distance
    distanceUltrasonic = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

    if ( debugresponse ) { 
      Serial.print ( "Ultrasonic distance: " );
      Serial.println ( distanceUltrasonic );
    }
  
}



// Infared variables
int distanceIRLeft;
int distanceIRRight;

void loopInfrared() {
  float voltsL = analogRead(LeftInfared)*0.0048828125;  // value from sensor * (5/1024)
  distanceIRLeft = 13*pow(voltsL, -1); // worked out from datasheet graph
  
  float voltsR = analogRead(RightInfared)*0.0048828125;  // value from sensor * (5/1024)
  distanceIRRight = 13*pow(voltsR, -1); // worked out from datasheet graph
  
  if ( debugresponse ) { 
      Serial.print ( "Infared distances: Left: " );
      Serial.print ( distanceIRLeft );
      Serial.print ( " Right: " );
      Serial.println ( distanceIRRight );
    }
}



// Pressure variables
int pressureReadingLeft;      // the analog reading from the FSR resistor divider
int pressureReadingRight;

void loopPressure() {
  pressureReadingLeft = analogRead(LeftHandbrake);
  pressureReadingRight = analogRead(RightHandbrake);
  
  if ( debugresponse ) { 
      Serial.print ( "Pressure reading: Left: " );
      Serial.print ( pressureReadingLeft );
      Serial.print ( " Right: " );
      Serial.println ( pressureReadingRight );
    }

}

// ------- Sensor End -------



// Bluetooth Start

// Bluetooth End



// Nav Start

// Nav End




// ------- Serial Start -------


SoftwareSerial MicroCtrlIMU(MicroCtrlIMU_rx, MicroCtrlIMU_tx);
// Arduino Pin 12 goes to MicroCtrlIMU TX
// Arduino Pin 13 goes to MicroCtrlIMU RX

SoftwareSerial MicroCtrlBLE(MicroCtrlBLE_rx, MicroCtrlBLE_tx);
// Arduino Pin 12 goes to MicroCtrlIMU TX
// Arduino Pin 13 goes to MicroCtrlIMU RX

const unsigned int maxlength = 7;

char telemetry[maxlength];

//-----

String pad3(int input) { // shoves a few zeros on the head of a number
  if (input < 100) {
    if (input < 10) {
      return "00" + String(input);
    } else {
      return "0" + String(input);
    }
  } else {
    return String(input);
  }
}

int between(int input, int low, int high){ // self explanatory
  if ( input > high ) { return high; }
  else if ( input < low ) { return low; }
  else { return input; }
}


void listBytes() { // lists the individual bytes of a message
  for (int i = 0; i < (maxlength - 1); i++) {
    String indivbyte;
    indivbyte = "Byte No. " + String(i) + " Is " + telemetry[i];
    Serial.println(indivbyte);
  }
}



float getBLERSSI() {

  String RSSIString;
  int i;

  if (telemetry[2] == '-') {
    i = 2;
  } else {
    i = 3;
  }
  //If the minus sign is there, include it. Else, cut it off (start on next index over)

  while (i < 6) {
    RSSIString = RSSIString + telemetry[i];  // built string out of character array
    i++;
  }

  return ("%0.2f", (RSSIString.toFloat()));  // Format for vesc motors
}



float getIMUTelem() {

  String IMUString;
  int i;

  if (telemetry[2] == '-') {
    i = 2;
  } else {
    i = 3;
  }
  //If the minus sign is there, include it. Else, cut it off (start on next index over)

  while (i < 6) {
    IMUString = IMUString + telemetry[i];  // built string out of character array
    i++;
  }

  return ("%0.2f", (IMUString.toFloat()));  // Format for vesc motors
}



void parseSerialTelemetry() {
      if (debugresponse) {
        Serial.print("Command Got!: ");
        Serial.println(telemetry);
      }
      // listBytes();
      //---------------------------


      bool didexecute = true;

      switch (telemetry[0]) {  // Case by case basis of the Purpose Byte
        case '9':
          if (debugresponse) { Serial.println("Purpose: Motor telemetry"); }
          if (telemetry[2] == '+' || telemetry[2] == '-') {
            if (debugresponse) { Serial.println("Action: Read RPM"); }
            float rpm = getMotorRPM();
            if (debugresponse) {
              Serial.print("Location: ");
              Serial.println(telemetry[1]);
              Serial.print("Motor RPM: ");
              Serial.println(rpm);
            }
          } else {
            if (debugresponse) { Serial.println("Action: Unrecognized"); }
            didexecute = false;
          }
          break;

        case '8':
          if (debugresponse) { Serial.println("Purpose: BLE telemetry"); }
          if (telemetry[2] == '+' || telemetry[2] == '-') {
            if (debugresponse) { Serial.println("Action: Read RSSI"); }
            float rssi = getBLERSSI();
            if (debugresponse) {
              Serial.print("Location: ");
              Serial.println(telemetry[1]);
              Serial.print("BLE RSSI: ");
              Serial.println(rssi);
            }
          } else {
            if (debugresponse) { Serial.println("Action: Unrecognized"); }
            didexecute = false;
          }

          break;
        case '7':
          if (debugresponse) { Serial.println("Purpose: IMU telemetry"); }
          if (telemetry[2] == '+' || telemetry[2] == '-') {
            if (debugresponse) { Serial.println("Action: Read IMU"); }
            float imu_telem = getIMUTelem();
            if (debugresponse) {
              Serial.print("Location: ");
              Serial.println(telemetry[1]);
              Serial.print("IMU: ");
              Serial.println(imu_telem);
            }
          } else {
            if (debugresponse) { Serial.println("Action: Unrecognized"); }
            didexecute = false;
          }
          // -----

          break;  // End of motor control

        default:
          if (debugresponse) { Serial.println("Purpose: Unrecognized"); }
          didexecute = false;
          /* ------------------------------------------------------ *
        *  Purpose byte, default case: Unknown Command
        * ------------------------------------------------------ */
      }
}

// ------- Serial End -------



int spd_count = 0;
bool increase_spd = true;
bool decrease_spd = false;

int telem_instance_count = 0;



void setup() {

  pinMode(UltrasonicTrig, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(UltrasonicEcho, INPUT); // Sets the echoPin as an INPUT

  Serial.begin(9600);
  delay(500);

  MicroCtrlBLE.begin(9600);
  delay(500);

  Serial.println("Mega Master Start!");

  
  vescSerial1.begin(9600);
  vescM1.setSerialPort(&vescSerial1);

  vescSerial2.begin(9600);
  vescM2.setSerialPort(&vescSerial2);

  
}



void loop() {

  // GET SENSOR DATA

    loopUltrasonic();
    loopInfrared();
    loopPressure();

    delay(100);

  // SEND MOTOR COMMANDS

  const int maxspeed = 5;
  //static int i;

  int targetL = maxspeed;
  int targetR = maxspeed;
  
  static int L = 0;
  static int R  = 0;


  /*
   * Reasonable stopping distances
   * Ultrasonic: 60 (5 inches)
   * Pressure readings: 300 (jumps between 10 and 1000 no to full grip)
   * RSSI: -40 (-90 furthest to -30 closest)
   * Infared: 40 (6 inches?)
   * 
   */


  
  if ( distanceUltrasonic < 60 || pressureReadingLeft < 300 ) { // || getBLERSSI() > -40
    targetL = 0; 
    targetR = 0; 
  }

  if ( distanceIRLeft < 40 ) { 
    targetL = 0; 
  }

  if ( distanceIRRight < 40 ) { 
    targetR = 0; 
  }

  
  L = L + between( (targetL - L), -2, 1 ); // Tries to go to target speed, limited in set increments. Works with decimals and not-nice fractions.
  R = R + between( (targetR - R), -2, 1 ); // Can slow to a stop faster then it speeds back up. 

  if (debugresponse) {
  
  Serial.print ("Left target speed is ");
  Serial.print (targetL);
  Serial.print (" , left motor is running at ");
  Serial.println (L);
  Serial.println ("");

  }
  
  setMotorSpeed(1, L);
  delay(75);

  setMotorSpeed(2, R);
  delay(75);


  // ACQUIRE TELEMETRY
  delay(300);



telem_instance_count = telem_instance_count + 1;
if (telem_instance_count < 3){
  MicroCtrlMotor.listen();
  while (MicroCtrlMotor.available() > 0) {
    static unsigned int tlm_pos = 0;
    char inByte = MicroCtrlMotor.read();
    if (inByte != '\n' && (tlm_pos < maxlength - 1)) {
      telemetry[tlm_pos] = inByte;
      tlm_pos++;
    }
    else {
      telemetry[tlm_pos] = '\0';
      parseSerialTelemetry();
      Serial.println("");
      tlm_pos = 0;  // Await next command
    }
  }
}  
else if(telem_instance_count < 6){
  MicroCtrlBLE.listen();
  while (MicroCtrlBLE.available() > 0) {
    static unsigned int tlm_pos = 0;
    char inByte = MicroCtrlBLE.read();
    if (inByte != '\n' && (tlm_pos < maxlength - 1)) {
      telemetry[tlm_pos] = inByte;
      tlm_pos++;
    }
    else {
      telemetry[tlm_pos] = '\0';
      parseSerialTelemetry();
      Serial.println("");
      tlm_pos = 0;  // Await next command
    }
  }
} else {
  telem_instance_count = 0;
}
  
}
