/*
 * CORRECT EDITION 4/2/23
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

  Key: '-' indicates notes, '*' indicates issues that should be looked at
  
  ---
  
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

    
  // Pins
    // Analog
      int LeftHandbrake = 14;   // A0
      int RightHandbrake = 15;  // A1
      int LeftInfared = 16;     // A2
      int RightInfared = 17;    // A3
      //int ANALOG4 = 18; // A4
      //int ANALOG5 = 19; // A5

    // Digital and Serial

      int UltrasonicTrig = 27;
      int UltrasonicEcho = 28;

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
        
        tlmL[tlm_pos] = '\0'; //close buffer (IS THIS NECCESSARY FOR INTERNAL BUFFER???)

        // ---

          if (tlmL[2] == '+' || tlmL[2] == '-') { //If thing is valid
  
            String RSSIString; //start up buffer string
            int i; if (tlmL[2] == '-') { i = 2; } else { i = 3; } //adjust index to include / exclude sign
            while (i < 6) {
              RSSIString = RSSIString + tlmL[i];  // built string out of character array
              i++;
            }
            //rssiL = ("%0.2f", ( RSSIString.toFloat() ) ); 

            rssiL = ( atoi( RSSIString.c_str() ) );

            if (debugresponse) { Serial.println(rssiL); }
          
          } else {} //invalid
        
        // ---
        
        tlm_pos = 0;  // Await next command
        
      }   }   }

// ---===---

   void getRSSIR() { //Comprehensive acquisition of RSSI

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

            if (debugresponse) { Serial.println(rssiR); }
          
          } else {} //invalid
        
        // ---
        
        tlm_pos = 0;  // Await next command
        
      }   }   }

// ---===---

      void getRSSIF() { //Comprehensive acquisition of RSSI

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

            if (debugresponse) { Serial.println(rssiF); }
          
          } else {} //invalid
        
        // ---
        
        tlm_pos = 0;  // Await next command
        
      }   }   }
          

// ------- Bluetooth Serial End -------



// ------- LoRa Start -------
// ------- LoRa End -------


// ------- Control Start -------

    int threshUltrasonic = 60; // (reading of about 5 inches)
    int threshPressure = 500; // (From about 10, nothing, to about 1000, full grip)
    int threshRSSI = -40; // (From -90, furthest, to -30, closest) VERIFY
    int threshIR = 40; // (6 inches?) 

  // Demonstration Contrller
   
    void DemoControl(){
      
      const int maxspeed = 5;

      static int L = 0; // Establish speed variables
      static int R  = 0;

      int targetL = maxspeed; // Start by trying to run motors at full
      int targetR = maxspeed;

      // Target Shifting
        if ( distanceUltrasonic < threshUltrasonic || pressureReadingLeft < threshPressure || pressureReadingRight < threshPressure ) { // Anything to stop both motors
          targetL = 0; 
          targetR = 0; 
        }

        if ( distanceIRLeft < threshIR ) { 
          targetL = 0; 
        }

        if ( distanceIRRight < threshIR ) { 
          targetR = 0; 
        }
      //
    
      L = L + between( (targetL - L), -2, 1 ); // Tries to go to target speed, limited in set increments. Works with decimals and not-nice fractions.
      R = R + between( (targetR - R), -2, 1 ); // Discrepancy in absolute value of down/up increments is the relative rate of deccel / accel. Here, deccelerates twice as fast.

      setMotorSpeed(L, R);

      if (debugresponse) {
      
        Serial.print ("Left target speed is ");
        Serial.print (targetL);
        Serial.print (" , left motor is running at ");
        Serial.println (L);
        Serial.println ("");

      }
    }

    
  
   void FacilityControl(){
      
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
      
        Serial.print ("Target speed is ");
        Serial.print (target);
        Serial.print (" , Motor duty is set to ");
        Serial.println (Speed);
        Serial.println ("");

      }
    }

  // ---

// ------- Control End -------



// ------- Nav Start -------
// ------- Nav End -------



void setup() {

  pinMode(UltrasonicTrig, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(UltrasonicEcho, INPUT); // Sets the echoPin as an INPUT

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

}



void loop() {

  // GET SENSOR DATA
  // loopUltrasonic();
  // loopInfrared();
  // loopPressure();

    delay(100);
  //



  //=====---===== CONTROLLER =====---=====

  //FacilityControl();

  getRSSIL(); 1`
  Serial.println(rssiL);
 
  //=====---===== CONTROLLER =====---=====
  


  // ACQUIRE TELEMETRY
  delay(300);

  /*
    SoftSerialBLE.listen();
    while (SoftSerialBLE.available() > 0) {
      static unsigned int tlm_pos = 0;
      char inByte = SoftSerialBLE.read();
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
*/


/*

  telem_instance_count = telem_instance_count + 1;
 
  if(telem_instance_count < 6){
    
  } else {
    telem_instance_count = 0;
  }

 */
  
}
