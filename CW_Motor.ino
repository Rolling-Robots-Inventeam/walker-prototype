

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

//-------------------------

//#include <bluefruit.h>
#include <VescUart.h>
#include <SoftwareSerial.h>
#include <stdlib.h>

//-------------------------

 bool debugresponse = true;


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
int M1tx = 13; //tx is white
int M1rx = 12; //rx is purple
//int DIGITAL11= 11;
//int DIGITAL10 = 10;
//int DIGITAL9 = 9;
//int DIGITAL7 = 7;
//int DIGITAL5 = 5;



// Motor Integration Start

VescUart vescM1;
SoftwareSerial vescSerial(M1rx, M1tx);
 
//--------

  /*
void readUART() {
  if (vesc.getVescValues()) {

    Serial.print("RPM: "); Serial.println(vesc.data.rpm);
    Serial.print("inpVoltage: "); Serial.println(vesc.data.inpVoltage);
    Serial.print("ampHours: "); Serial.println(vesc.data.ampHours);
    Serial.print("tachometerAbs: "); Serial.println(vesc.data.tachometerAbs);

  }
  else
  {
    Serial.println("Failed to get data!");
  }
}

  */

// Motor Integration End 







// UART Management Start



const unsigned int maxlength = 7;

char command [maxlength];

// 1 Purpose byte (Master > motor (drive controlling) or motor > master (sensor reading) )
// 1 Location byte (Which device in the above category (which # motor, which sensor, etc.))
// 3 Action bytes (What to do? Add some speed? Stop motors? Get X data?)
// 1 Tail byte (for null character message ender)


//--------

void listBytes(){
 for(int i=0; i<(maxlength-1); i++){
        String indivbyte;
        indivbyte = "Byte No. " + String(i) + " Is " + command[i];
        Serial.println(indivbyte);
      }
}

//--------

float getMotorDuty(){
  
    String DutyString;
    int i;
    
    if( command[2] == '-' ){i = 2;} else {i = 3;} 
    //If the minus sign is there, include it. Else, cut it off (start on next index over)

    while(i < 6){
      DutyString = DutyString + command[i]; // built string out of character array
      i++;    
    }

  return( "%0.2f", ( DutyString.toFloat() / 100 ) ); // Format for vesc motors
  
}

//--------

// UART Management End








void setup() {

 Serial.begin(9600);

 vescSerial.begin(9600);
 vescM1.setSerialPort(&vescSerial);

 Serial.println("Started!");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);


}





void loop() {

while (Serial.available() > 0){

  //---------------------------

  // ACQUIRE COMMAND

  static unsigned int cmd_pos = 0;

  char inByte = Serial.read();

  if ( inByte != '\n' && ( cmd_pos < maxlength - 1) ) { 
    command[cmd_pos] = inByte;
    cmd_pos++;

  }

  //-------------
  
  else {
    
    command[cmd_pos] = '\0';
    
    if(debugresponse) { 
      Serial.print("Command Got!: "); 
      Serial.println(command);
    }
   

  //---------------------------

    
/*
 * Everything hereafter happens whenever a command is recieved
 */
  
      bool didexecute = true;
    
      switch(command[0]){ // Case by case basis of the Purpose Byte

      /* ------------------------------------------------------ *
       * 
       *  KEY for Command Index 0 ( Purpose Byte )
       *  
       *  1 -> send command of some sort to some motor
       *  2 -> send command of some sort to some sensor
       *  
       *  Remember:  Case comparisons must be made with character type
       * 
       * ------------------------------------------------------ */


     
      case '1': 
       /* ------------------------------------------------------ *
        *  Purpose byte, case 1: Commanding Motors
        * ------------------------------------------------------ */
    
        if(debugresponse) { Serial.println("Purpose: Motor Command"); }

        /* ------------------------------------------------------ *
         * 
         *  KEY for Command Index 2 when Commanding Motors (Header of Action bytes)
         *  
         *  Implemented:
         *  + -> Set motor duty, positive value
         *  - -> Set motor duty, negative value
         *  
         *  Unimplemented:
         *  / -> Disengage motors
         *  ! -> Engage motors
         * 
         * ------------------------------------------------------ */

        if (command[2] == '+' || command[2] == '-'){ // If command is to set the motor duty
          
          if(debugresponse) { Serial.println("Action: Set Duty"); }

          float Duty = getMotorDuty();

          if(debugresponse) { 
            Serial.print("Location: ");
            Serial.println(command[1]);
            Serial.print("Motor duty: ");
            Serial.println(Duty);
          }

          switch(command[1]){ // Location byte (motors), choose which motor to send to
            case '1':
              vescM1.setDuty(Duty);
              if(debugresponse) { Serial.println("M1 Duty Set!"); }
            break;

            case '2':
              //vescM2.setDuty(Duty);
              if(debugresponse) { Serial.println("M2 Duty Set!"); }
            break;
            
            case '3':
              //vescM3.setDuty(Duty);
              if(debugresponse) { Serial.println("M3 Duty Set!"); }
            break;
            
            case '4':
              //vescM4.setDuty(Duty);
              if(debugresponse) { Serial.println("M4 Duty Set!"); }
            break;

            default: 
              if(debugresponse) { Serial.println("Location: Unrecognized"); }
              didexecute = false;      
          }

        } // if statement is to set motor duty


        else if (command[2] == '/') { // If command is to disengage motors
           if(debugresponse) { Serial.println("Action: Disengage"); }
           // do whatever disengage entails
        }

        else {
          if(debugresponse) { Serial.println("Action: Unrecognized"); }
          didexecute = false;
        }

        // -----
     
      break; // End of motor control



     
  
      case '2':
       /* ------------------------------------------------------ *
        *  Purpose byte, case 2: Commanding Sensors
        * ------------------------------------------------------ */
      
        if(debugresponse) { Serial.println("Purpose: Sensor Command"); }

      break; // End of sensor control

      

      default: 
        if(debugresponse) { Serial.println("Purpose: Unrecognized"); }
        didexecute = false;
       /* ------------------------------------------------------ *
        *  Purpose byte, default case: Unknown Command
        * ------------------------------------------------------ */
      } 
    
 
  

  //---------------------------

  if (didexecute == true)  { // If it works, blinks solid for 1 second
    
    
    if(debugresponse) { Serial.println("Command executed!"); }

    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    
  }
  
  else { 
    
    if(debugresponse) { Serial.println("Command failed to execute!"); }
    
    for(int i=0; i<6; i++){
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(165);
      }
  }



  //---------------------------

  Serial.println(""); 
  cmd_pos = 0; // Await next command
  
  }
  
  } 
 
}
