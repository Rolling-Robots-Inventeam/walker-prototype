

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


void setup() {

 

}

void loop() {

}
