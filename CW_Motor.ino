

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
  *  MOTOR CHIP
  *  This chip handles the entirety of motor management.
  *   - Individual ports per motor
  *   - Takes encoder data
  *   - Serial connection to master
  */


//------------------------

// Pins

// Analog
//int ANALOG0 = 14;
//int ANALOG1 = 15;
//int ANALOG2 = 16;
//int ANALOG3 = 17;
//int ANALOG4 = 18;
//int ANALOG5 = 19;

// Digital
int LeftDrive = 13;
int RightDrive = 12;
//int DIGITAL11 = 11;
//int DIGITAL10 = 10;
//int DIGITAL9 = 9;
//int DIGITAL7 = 7;
//int DIGITAL5 = 5;

//-------------------------




// Motor Integration Start

  const int bounds[2] = {0, 100};

  #include <Servo.h>

  Servo driveR;
  Servo driveL;

  void DriveSetup(){
    driveR.attach(RightDrive, 1000, 2000);
    driveL.attach(LeftDrive, 1000, 2000);
  }

  void Drive(double Left, double Right){
    driveR.writeMicroseconds(map(Left, bounds[0], bounds[1], 1000, 2000));
    driveL.writeMicroseconds(map(Right, bounds[0], bounds[1], 1000, 2000));
  }

// Motor Integration End




// Serial Management Start



void SerialSetup(){
  
}



// Serial Management End


void setup() {

 

}

void loop() {

}
