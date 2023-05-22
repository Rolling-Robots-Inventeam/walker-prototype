

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
  *  MASTER CHIP
  *  This chip handles the majority of processes.
  *   - Sensor data, including handbrake and various distance sensors
  *   - All Bluetooth tracking data
  *   - Hosts navigation algorithm
  *   - Serial connection to motor chip
  */



// -----

#include <bluefruit.h>

// -----



//------------------------

// Pins

// Analog
int Handbrake = 14;
int LeftInfared = 15;
int RightInfared = 16;
//int ANALOG3 = 17;
//int ANALOG4 = 18;
//int ANALOG5 = 19;

// Digital
int UltrasonicTrig = 13;
int UltrasonicEcho = 12;
//int DIGITAL11 = 11;
//int DIGITAL10 = 10;
//int DIGITAL9 = 9;
//int DIGITAL7 = 7;
//int DIGITAL5 = 5;

// Misc
//int SWITCHINTERNAL = 4;
//int LIGHTINTERNAL = 3;

//-------------------------


// Sensor - Handbrake Start

// Sensor - Handbrake End



// Sensor - Infared Start

// Sensor - Infared End


// Sensor - Ultrasonic Start

// Sensor - Ultrasonic End


// Bluetooth Start

// Bluetooth End



// Nav Start

// Nav End




// Serial Start


String pad3( int input ){
  if ( input < 100) {
    if ( input < 10) { return "00" + String(input);
    } 
    else { return "0" + String(input); }
  } 
  else { return String(input); }
}



void setMotorSpeed(int motor, int spd){ // -100 to 100
  // Motor side does the formatting, saves us an unnecessary decimal point in char

  String cmd;  // initialize

  /*
   *  Example motor command:
   *  to set Motor 2 to 0.28 -> 12+028
   *  
   *  1 <- Purpose of this command (to control motor)
   *  2 <- Which motor (2)
   *  + <- Type of action (set duty to some positive value)
   *  0 \
   *  2  |<- Data needed to execute command (What positive value to set duty to) (Integer -100 to 100, padded with 0s to 3 places)
   *  8 /
   * 
   */
  
  if (spd<0){
    cmd = "1" + String(motor) + "-" + pad3( abs (spd) );
    //   Purpose + Location + Action + Data
  }
  else{
    cmd = "1" + String(motor) + "+" + pad3( abs (spd) );
    //   Purpose + Location + Action + Data
  }
  
  // Adafruit hates writing strings over serial, so here we handconvert string into char array
  char charcommand[cmd.length()];
  
  for (int i = 0; i < cmd.length(); i++){
    charcommand[i] = cmd.charAt(i);
  }
  Serial.write(charcommand);
  Serial.println(" ");
}


// Serial End





void setup() {

Serial.begin(9600);

}



void loop() {

  /*
  static int i = -100;

  setMotorSpeed(1, i);
  i = i + 10;
  delay(1000);

  if( i > 100 ) {
    i = -100;
  }
  */

  setMotorSpeed(1, 30);
  delay(2000);
  setMotorSpeed(1, -30);
  delay(2000);
  
}
