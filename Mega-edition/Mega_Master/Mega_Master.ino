/*
 * CORRECT EDITION 4/23/23
 */


  /*
  *  MEGA MASTER CHIP
  *  This chip handles almost every process.
  *   - Sensor data, including handbrake and various distance sensors
  *   - Getting BLE data over serial links
  *   - Hosts navigation algorithm
  *   - Motor controlling via vesc serials
*/

/* --- Latest Update Log --- 

  Date: 4/23/23 

  Alex. Adding the debug button. It puts it in debug / disable !

  Date: 4/16/23

  Alex. Trying to add Vesc data.

  Date: 4/2/23

  Alex. We're doing this thing.
  - Expand the RSSI function to work with all three serial ports.
  - Isolate redundant telemetry functions. Comment out until it's verified the code works: then remove.
  - Incorporate actuators? into hardware scheme.
  - Start on Leo's nav algorithm.
  
  Date: 3/19/23
  
  Alex. Doing channel allocations.

  Date: 3/15/23

  Alex here! Just finished rewiring this thing for the Mega. Wiped out all the UART communication save for BLE, and brought over all the motor control stuff, now attached to hardware serials.

  IMPORTANT THING: we have 3 BLE sensors we should be tracking, and this program's telemetry just accounts for 1!
    - The other two telemetry updates should go in the bottom, next to the first.
    - Remember that two are hardware serials and one is a software serial!
    * Should the telemetries all update at once, or should they stagger over a period?
    * Could we expand the existing UART framework to accomodate commands from the 3 sensors? I.E. a case switcher that checks Purpose byte, and each BLE gets its own Purpose number
*/


// ------- Setup Start -------
  // Includes
    #include <VescUart.h>
    #include <SoftwareSerial.h>
    #include <stdlib.h>
  
  // Constants
    bool debugresponse = true;
    int vescbaudrate = 9600;

  // Variables

    int spd_count = 0;
    bool increase_spd = true;
    bool decrease_spd = false;

    int telem_instance_count = 0;

    int rssiF = 0;
    int rssiL = 0;
    int rssiR = 0;


    bool disable() {
      return (!digitalRead(22) );
    }

    
  // Pins
    // Analog
      const int LeftHandbrake = 14;   // A0
      const int RightHandbrake = 15;  // A1
      const int LeftInfared = 16;     // A2
      const int RightInfared = 17;    // A3
      //int ANALOG4 = 18; // A4
      //int ANALOG5 = 19; // A5

    // Digital and Serial

      const int UltrasonicTrig = 27;
      const int UltrasonicEcho = 28;
      const int FOR_RELAY_PIN = 3;
      const int REV_RELAY_PIN = 4;

      const int DebugSwitchPin = 22;

        /* All parenthesis in rx -> tx order
        - Serial (0, 1)   - Serial1 (19, 18)   - Serial2 (17, 16)   - Serial3 (15, 14) 
          Motors:
           Left: Serial2
           Right: Serial3
          BLE Sensors: 
           No. 1: Serial0 (Front)
           No. 2: Serial1 (Right)
           No. 3: Bonus software serial (22, 23) (Left)
          LoRa:
           Software serial (30,31)

         Only software serial pins have to be specified. All others are set by default by Arduino */
    
      SoftwareSerial SoftSerialBLE(52, 53);
      SoftwareSerial SoftSerialLoRa(50, 51);

// ------- Setup End -------



// ------- Motor Start -------

  VescUart vescML;
  VescUart vescMR;

  int oldSpeedL = 0;
  int oldSpeedR = 0;

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


  void setMotorSpeed(float left, float right){
    vescML.setDuty(left / 100);
    vescMR.setDuty(right / 100); 
  }

  void setMotorSpeedBetter(float left, float right){
    if (left != oldSpeedL) {
  
      vescML.setDuty(left / 100);
      oldSpeedL = left;

      if ( debugresponse ) { 
        Serial.print ( "setMotorSpeed: Left speed changed: " );
        Serial.println ( left );
      } 
     
    } else {
        Serial.println ( "setMotorSpeed: Left speed unchanged" );
    }
    
    
    if (right != oldSpeedR) {
      vescMR.setDuty(right / 100);
      oldSpeedR = right;

      if ( debugresponse ) { 
        Serial.print ( "setMotorSpeed: Right speed changed: " );
        Serial.println ( right );
      } 
     
    } else {
        Serial.println ( "setMotorSpeed: Right speed unchanged" );
    }}

    void motordebug(){
      if(debugresponse){
    
        if (vescML.getVescValues() ){
           Serial.print("Left RPM: ");
           Serial.print(vescML.data.rpm);
           Serial.print(" | Tachometer: ");
           Serial.println(vescML.data.tachometerAbs);
        }
        else { Serial.println("Left Data Failed!"); }
        
        if (vescMR.getVescValues() ){
           Serial.print("Right RPM: ");
           Serial.print(vescMR.data.rpm);
           Serial.print(" | Tachometer: ");
           Serial.println(vescMR.data.tachometerAbs);
        }
        else { Serial.println("Right Data Failed!"); }
    
      } else{}
    }

// ------- Motor End -------



// ------- Actuator Start -------

void extendActuators(){
  digitalWrite(FOR_RELAY_PIN, HIGH);
  digitalWrite(REV_RELAY_PIN, LOW);
}

void retractActuators(){
  digitalWrite(FOR_RELAY_PIN, LOW);
  digitalWrite(REV_RELAY_PIN, HIGH);
}

// ------- Actuator End -------



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
      Serial.print ( "loopUltrasonic: Ultrasonic distance: " );
      Serial.println ( distanceUltrasonic );
    }
  
  }



  // Infared variables
  int distanceIRLeft;
  int distanceIRRight;

  void loopIR() {
    float voltsL = analogRead(LeftInfared)*0.0048828125;  // value from sensor * (5/1024)
    distanceIRLeft = 13*pow(voltsL, -1); // worked out from datasheet graph
  
    float voltsR = analogRead(RightInfared)*0.0048828125;  // value from sensor * (5/1024)
    distanceIRRight = 13*pow(voltsR, -1); // worked out from datasheet graph
  
    if ( debugresponse ) { 
      Serial.print ( "loopIR - Infared distances: Left: " );
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
      Serial.print ( "loopPressure - Pressure reading: Left: " );
      Serial.print ( pressureReadingLeft );
      Serial.print ( " Right: " );
      Serial.println ( pressureReadingRight );
      }

  }

// ------- Sensor End -------



// ------- Bluetooth Serial Start -------

  const unsigned int maxlength = 7;

  char telemetry[maxlength];

  char tlmF[maxlength];
  char tlmL[maxlength];
  char tlmR[maxlength];

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

// ---===---

/*
 RSSI Functions:
 - Simply run em' once in the update cycle (ideally)
 - They output to global variables 'rssiL', 'rssiR', and 'rssiF'
 - use variables at your discretion
*/

// ---===---

  void getRSSIL() { //Comprehensive acquisition of RSSI

    // --- DEVELOPMENT OF INFO ---
    // Serial command (Individual bytes)
    // TLM buffer (Full length command)
    // RSSIString (Extracted RSSI value, string form)
    // rssi (RSSI value, float form)
    
    //Serial.listen(); //find serial value

    while (SoftSerialBLE.available() > 0) { //If there are available bytes...
      
      static unsigned int tlm_pos = 0; //start at index zero
       
      char inByte = SoftSerialBLE.read(); //get the next byte
      if (inByte != '\n' && (tlm_pos < maxlength - 1)) { //if it's before we reach the end...
        tlmL[tlm_pos] = inByte; //add to buffer
        tlm_pos++; //set index up
      }
      else { //if we're at the end...
        
        tlmL[tlm_pos] = '\0'; //close buffer (IS THIS NECCESSARY FOR INTERNAL BUFFER???) (a.k.a with known length)

        // --- 

          if (tlmL[2] == '+' || tlmL[2] == '-') { //If thing is valid
  
            String RSSIString; //start up buffer string
            
            int i; if (tlmL[2] == '-') { i = 2; } else { i = 3; } // Create index, adjust index to include / exclude sign
            while (i < 6) { 
              RSSIString = RSSIString + tlmL[i];  // built string out of character array
              i++; // move index up
            }

            rssiL = ( atoi( RSSIString.c_str() ) ); // convert the string into an integer

            if (debugresponse) { 
              Serial.print("GetRSSIL - Value: ");
              Serial.println(rssiL); 
              }

          } else {} //invalid
        
        // ---
        
        tlm_pos = 0;  // Await next command
        
      }   }   }

// ---===---

   void getRSSIR() { 

    while (Serial1.available() > 0) { //If there are available bytes...
      
      static unsigned int tlm_pos = 0; //start at index zero
       
      char inByte = Serial1.read(); //get the next byte
      if (inByte != '\n' && (tlm_pos < maxlength - 1)) { //if it's before we reach the end...
        tlmR[tlm_pos] = inByte; //add to buffer
        tlm_pos++; //set index up
      }
      else { //if we're at the end...
        
        tlmR[tlm_pos] = '\0'; //close buffer (IS THIS NECCESSARY FOR INTERNAL BUFFER???)

        // ---

          if (tlmR[2] == '+' || tlmR[2] == '-') { //If thing is valid
  
            String RSSIString; //start up buffer string
            int i; if (tlmR[2] == '-') { i = 2; } else { i = 3; } //adjust index to include / exclude sign
            while (i < 6) {
              RSSIString = RSSIString + tlmR[i];  // built string out of character array
              i++;
            }
           
            rssiR = ( atoi( RSSIString.c_str() ) );

            if (debugresponse) { 
              Serial.print("GetRSSIR - Value: ");
              Serial.println(rssiR); 
              }

          } else {} //invalid
        
        // ---
        
        tlm_pos = 0;  // Await next command
        
      }   }   }

// ---===---

   void getRSSIF() { 

    while (Serial.available() > 0) { //If there are available bytes...
      
      static unsigned int tlm_pos = 0; //start at index zero
       
      char inByte = Serial.read(); //get the next byte
      if (inByte != '\n' && (tlm_pos < maxlength - 1)) { //if it's before we reach the end...
        tlmF[tlm_pos] = inByte; //add to buffer
        tlm_pos++; //set index up
      }
      else { //if we're at the end...
        
        tlmF[tlm_pos] = '\0'; //close buffer (IS THIS NECCESSARY FOR INTERNAL BUFFER???)

        // ---

          if (tlmF[2] == '+' || tlmF[2] == '-') { //If thing is valid
  
            String RSSIString; //start up buffer string
            int i; if (tlmF[2] == '-') { i = 2; } else { i = 3; } //adjust index to include / exclude sign
            while (i < 6) {
              RSSIString = RSSIString + tlmF[i];  // built string out of character array
              i++;
            }
           
            rssiF = ( atoi( RSSIString.c_str() ) );

            if (debugresponse) { 
              Serial.print("GetRSSIF - Value: ");
              Serial.println(rssiF); 
              }
          
          } else {} //invalid
        
        // ---
        
        tlm_pos = 0;  // Await next command
        
      }   }   }
          

// ------- Bluetooth Serial End -------



// ------- LoRa Start -------
// ------- LoRa End -------


// ------- Control Start -------

  void UpdateData(){
    loopUltrasonic();
    loopIR();
    loopPressure();
    getRSSIF();
    getRSSIL();
    getRSSIR();
  }

    int threshUltrasonic = 60; // (reading of about 5 inches)
  
  void Debugger(){
      
      const int maxspeed = 6;

      static int Speed  = 0;

      int target = 0;

      // Target Shifting
        if ( distanceUltrasonic > threshUltrasonic ) { // Anything to stop both motors
          target = maxspeed; 
        }
      
    
      Speed = Speed + between( (target - Speed), -2, 1 );
    
      setMotorSpeed(Speed, Speed);

      if (debugresponse) {
      
        Serial.print ("Debug Controller - Target speed is ");
        Serial.print (target);
        Serial.print (" , Motor duty is set to ");
        Serial.println (Speed);
        Serial.println ("");

      }
    }

// ------- Control End -------



// ------- Nav Start -------








// ------- Nav End -------



void setup() {

  pinMode(UltrasonicTrig, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(UltrasonicEcho, INPUT); // Sets the echoPin as an INPUT

  pinMode(FOR_RELAY_PIN, OUTPUT);
  pinMode(REV_RELAY_PIN, OUTPUT);

  pinMode(DebugSwitchPin, INPUT_PULLUP);

  // Motor Setup
    Serial2.begin(vescbaudrate);
    vescML.setSerialPort(&Serial2);

    Serial3.begin(vescbaudrate);
    vescMR.setSerialPort(&Serial3);
  
  // BLE Setup
    Serial.begin(9600);
    Serial1.begin(9600);
    SoftSerialBLE.begin(9600);

  // LoRa Setup
    SoftSerialLoRa.begin(9600);

  Serial.println("Mega Master Start!");

  debugresponse = true;

}


   bool breakout;
   int reason; // it switches to several other loops.
    // 1 is room nav
    // 2 is debug looping
    // Anything else does nothing, and maybe logs an error message for now.

    // After the reason is taken care of, break is reset, and the main business comes


// --- === ---
void loop() {
// --- === ---

 
 
  
 while (breakout == false) { // Run the user navigation

  Serial.println("I am stuck in the loop");
    UpdateData();
    motordebug();
    

    setMotorSpeed(-5,-5);
    

    if(disable()) { 
      breakout = true; 
      reason = 2; 
      if(debugresponse){ Serial.println("Switch over!!"); }
      }
    
    
  delay(500);

 }
 

 if(reason == 1){

  // Navigation


/*
 * Variable Key:
 * 
 * Ultrasonic distance at the front: distanceUltrasonic
 * Infared distance to left: distanceIRLeft
 * Infared distance to right: distanceIRRight
 * Left pressure sensor: pressureReadingLeft
 * Right pressure sensor: pressureReadingRight
 * RSSI for front: rssiF
 * RSSI for left: rssiL
 * RSSI for right: rssiR
 * 
 * Format for vesc motor telemetry: [motor channel].data.[type of information]
 * Motor channels (2): vescML, vescMR
 * Types of information (4): rpm, inpVoltage, ampHours, tachometerAbs
 * 
 */


  
    bool targetreached = false;
    while(targetreached == false){ delay(500); }

 }



  else if(reason == 2) { // Debugging
    while(disable() == 1){
      Serial.println("Debug mode!");
      //Debugger();
      delay(1000);
    }
    Serial.println("Get out of debug!");
  }



  else{ // What??
    Serial.println("How did we get here?");
    delay(1000);
    Serial.println("Short timeout for you.");
    delay(5000);
  }

  if(debugresponse){ Serial.println("Back to the top!"); } 
  breakout = false;
  reason = 0;

// --- === ---
}
// --- === ---
