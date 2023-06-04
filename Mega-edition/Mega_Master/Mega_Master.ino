


/*
 * CORRECT EDITION 5/28/23
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
 *  
 *  Alex here. Dealing with debug garbage.
 *  
 *  
 *  
 *  
 */


// ------- Setup Start -------
  // Includes
    #include <VescUart.h>
    #include <SoftwareSerial.h>
    #include <stdlib.h>
    #include <Adafruit_LSM6DSO32.h>
    
    
  
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
      const int LeftHandbrake = A0;
      const int RightHandbrake = A1;
      const int LeftInfared = A5;
      const int RightInfared = A4;

    // Digital and Serial

      const int LidarPin = 10;

      const int UltrasonicTrig = 28;
      const int UltrasonicEcho = 29;
      
      const int RelayLF = 32;
      const int RelayLR = 33;
      const int RelayRF = 34;
      const int RelayRR = 35;
      const int RelayBF = 36;
      const int RelayBR = 37;

      const int BumperPin = 40;
      const int SpeakerPin = 41;
      
      const int DebugSwitchPin = 49;

      bool debugdisable() {
        return (!digitalRead(DebugSwitchPin) );
      }

      bool bumped() {
        return (!digitalRead(BumperPin) );
      }

        /* All parenthesis in rx -> tx order
        - Serial (0, 1)   - Serial1 (19, 18)   - Serial2 (17, 16)   - Serial3 (15, 14) 
          Motors:
           Left: Serial2
           Right: Serial3
          BLE Sensors: 
           No. 1: Serial0 (Front)
           No. 2: Serial1 (Right)
           No. 3: Bonus software serial (52, 53) (Left)
          LoRa:
           Software serial (50,51)

         Only software ser  ial pins have to be specified. All others are set by default by Arduino */

      SoftwareSerial SoftSerialLoRa(50, 51);
      SoftwareSerial SoftSerialBLE(52, 53);


      void debugrespond ( String text ) {
        if(debugresponse){ Serial.println(text); }
      }


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
    digitalWrite(RelayLF, HIGH);
    digitalWrite(RelayRF, HIGH);
    digitalWrite(RelayLR, LOW);
    digitalWrite(RelayRR, LOW);
  }

  void retractActuators(){
    digitalWrite(RelayLR, HIGH);
    digitalWrite(RelayRR, HIGH);
    digitalWrite(RelayLF, LOW);
    digitalWrite(RelayRF, LOW);
  }

  void brake(int state){ //placehlolder for solenoid brakes on the walker
    if(state == 1){

    }
    else{

    }
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
      Serial.print ( voltsL );
      Serial.print ( " Right: " );
      Serial.println ( voltsR );
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

  int between(int input, int low, int high){ // self explanatory
    if ( input > high ) { return high; }
    else if ( input < low ) { return low; }
    else { return input; }
  }

    /*
    RSSI Functions:
    - Simply run em' once in the update cycle (ideally)
    - They output to global variables 'rssiL', 'rssiR', and 'rssiF'
    - use variables at your discretion
    */

  // getRSSI Left

   void getRSSIL() { //Comprehensive acquisition of RSSI

    // --- DEVELOPMENT OF INFO ---
    // Serial command (Individual bytes)
    // TLM buffer (Full length command)
    // RSSIString (Extracted RSSI value, string form)
    // rssi (RSSI value, float form)
    
    SoftSerialBLE.listen(); //find serial value

    if (SoftSerialBLE.available() == 0 && debugresponse) { Serial.println("GetRSSIL - Failed!"); } else {}
    
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
            int i; if (tlmL[2] == '-') { i = 2; } else { i = 3; } //adjust index to include / exclude sign
            while (i < 6) {
              RSSIString = RSSIString + tlmL[i];  // built string out of character array
              i++;
            }
            //rssiL = ("%0.2f", ( RSSIString.toFloat() ) ); 

            rssiL = ( atoi( RSSIString.c_str() ) );

            if (debugresponse) { 
              Serial.print("GetRSSIL - Value: ");
              Serial.println(rssiL); 
              }

          } else {} //invalid
        
        // ---
        
        tlm_pos = 0;  // Await next command
        
      }
    }
   }

  // getRSSI Right

   void getRSSIR() { 

    if (Serial1.available() == 0 && debugresponse) { Serial.println("GetRSSIR - Failed!"); } else {}
    
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
        
      }
    }
   }

  // getRSSI Forward

   void getRSSIF() { 

    if (Serial.available() == 0 && debugresponse) { Serial.println("GetRSSIF - Failed!"); } else {}

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
        
      }
    }
   }
          
// ------- Bluetooth Serial End -------



// ------- IMU Start -------

  // Basic demo for accelerometer & gyro readings from Adafruit
  // LSM6DSO32 sensor added

  // For SPI mode, we need a CS pin
  #define LSM_CS 12
  // For software-SPI mode we need SCK/MOSI/MISO pins
  #define LSM_SCK 13
  #define LSM_MISO 12
  #define LSM_MOSI 11

  float distance_x;

  Adafruit_LSM6DSO32 dso32;
  
  void IMUloop() {

    //   Get a new normalized sensor event 
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    dso32.getEvent(&accel, &gyro, &temp);

    float accel_x_tare;
    accel_x_tare = 0.5;
    float accel_x_tared;
    
  //  Serial.print("\t\tTemperature ");
  //  Serial.print(temp.temperature);
  //  Serial.println(" deg C");

    // Display the results (acceleration is measured in m/s^2) //
    if (debugresponse) {
    Serial.print("\t\tAccel X: ");
    Serial.print(accel.acceleration.x);
    Serial.print("\t\tAccel X Tared: ");
    }
    if (abs(accel.acceleration.x) > 0.2){
      accel_x_tared = accel.acceleration.x + accel_x_tare;
      } else {
        accel_x_tared = accel.acceleration.x;
        }
        
    if (debugresponse) { Serial.print(accel_x_tared); }

    
    if (abs(accel_x_tared) >= 0.2){
      distance_x = distance_x + (0.5 * (accel_x_tared) * pow(0.1, 2));
    }
  //  Serial.print(" \tY: ");
  //  Serial.print(accel.acceleration.y);
  //  Serial.print(" \tZ: ");
  //  Serial.print(accel.acceleration.z);
  if (debugresponse) {
    Serial.println(" m/s^2 ");

    Serial.print(distance_x);
    Serial.println(" m ");
  }

  //  // Display the results (rotation is measured in rad/s) //
  //  Serial.print("\t\tGyro X: ");
  //  Serial.print(gyro.gyro.x);
  //  Serial.print(" \tY: ");
  //  Serial.print(gyro.gyro.y);
  //  Serial.print(" \tZ: ");
  //  Serial.print(gyro.gyro.z);
  //  Serial.println(" radians/s ");
  //  Serial.println();

    delay(100);

    //  // serial plotter friendly format

    //  Serial.print(temp.temperature);
    //  Serial.print(",");

    //  Serial.print(accel.acceleration.x);
    //  Serial.print(","); Serial.print(accel.acceleration.y);
    //  Serial.print(","); Serial.print(accel.acceleration.z);
    //  Serial.print(",");

    // Serial.print(gyro.gyro.x);
    // Serial.print(","); Serial.print(gyro.gyro.y);
    // Serial.print(","); Serial.print(gyro.gyro.z);
    // Serial.println();
    //  delayMicroseconds(10000);
  }


  
// ------- IMU End -------



// ------- Control Start -------

  int RSSILavg;
  int RSSIRavg;
  int RSSIFavg;

  void UpdateData(){
    loopUltrasonic();
    loopIR();
    loopPressure();
    getRSSIF();
    getRSSIL();
    getRSSIR();
    motordebug();
    IMUloop();
  }
  
  void Debugger(){
      /*
      const int maxspeed = 6;

      static int Speed  = 0;

      int target = 0;

      // Target Shifting
        if ( distanceUltrasonic > Ultrasonic ) { // Anything to stop both motors
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
      
      */
    }


  void RSSIPlotter(){}
// ------- Control End -------

// ------- Nav Start -------


  void collisionDetect() {
    if (distanceUltrasonic > 80) {
      setMotorSpeed(-5,-5);  
      Serial.println("CollisionDetect - Driving!");
    } else {
      setMotorSpeed(0,0);
      Serial.println("CollisionDetect - Stopped!");
    }
  }

    static float ehL = 80; // Expected Highest. adaptive upper bound
    static float elL = 50; // Expected Lowest. adaptivel lower bound
    static float caL = 60; // Cumulative Average.

    static float ehR = 80; 
    static float elR = 50; 
    static float caR = 60;

  float mapRSSIL () { // Outputs 0-1. Input smoothing and advanced bound manipulation.
    
    float end; // result variable

    caL = (caL + abs(rssiL)) / 2; // smooth input with basic integral function


    if (caL > ehL){ // If it exceeds highest bound...
      end = 1; 
      ehL = caL;
      //elL++; // Since it's not being touched, bring lower bound up.
    }
    else if(caL < elL){
      end = 0;
      elL = caL;
      //ehL--;
    }
    else{
      end = (caL - elL) / (ehL - elL);
      //ehL--; elL++;
    }

    return (end);
  }

  float mapRSSIR () { // OUTPUTS ON 0-1 SCALE
    float end;

    caR = (caR + abs(rssiR)) / 2;

    if (caR > ehR){
      end = 1;
      ehR = caR;
      //elR++;
    }
    else if(caR < elR){
      end = 0;
      elR = caR;
      //ehR--;
    }
    else{
      end = (caR - elR) / (ehR - elR);
      //ehR--; elR++;
    }

    return (end);
  }

  void turnToBeacon() {
    float angle = mapRSSIR() - mapRSSIL();
    setMotorSpeed( angle * 5, -angle * 5);

  }

  

// ------- Nav End -------

// ------- Setup Function Start -------

  void setup() {

    
    debugresponse = true;
    

    pinMode(UltrasonicTrig, OUTPUT); // Sets the trigPin as an OUTPUT
    pinMode(UltrasonicEcho, INPUT); // Sets the echoPin as an INPUT

    pinMode(RelayLF, OUTPUT);
    pinMode(RelayLR, OUTPUT);
    pinMode(RelayRF, OUTPUT);
    pinMode(RelayRR, OUTPUT);
    pinMode(RelayBF, OUTPUT);
    pinMode(RelayBR, OUTPUT);

    // pinMode(LeftInfared, INPUT);
    // pinMode(RightInfared, INPUT);

    pinMode(SDA, INPUT);
    pinMode(SCL, INPUT);

    pinMode(SpeakerPin, OUTPUT);
    
    pinMode(BumperPin, INPUT_PULLUP);
    
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

    debugrespond("Mega Master Start!");


/*

    Serial.println("Adafruit LSM6DSO32 test!");
    
    if (!dso32.begin_I2C()) {
      if (!dso32.begin_SPI(LSM_CS)) {
      if (!dso32.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
      Serial.println("Failed to find LSM6DSO32 chip"); } } 
      while (1) {
        delay(10);
      }
    }

    Serial.println("LSM6DSO32 Found!");

    dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_8_G);
    Serial.print("Accelerometer range set to: ");
    switch (dso32.getAccelRange()) {
    case LSM6DSO32_ACCEL_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case LSM6DSO32_ACCEL_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case LSM6DSO32_ACCEL_RANGE_16_G:
      Serial.println("+-16G");
      break;
    case LSM6DSO32_ACCEL_RANGE_32_G:
      Serial.println("+-32G");
      break;
    }

    // dso32.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
    Serial.print("Gyro range set to: ");
    switch (dso32.getGyroRange()) {
    case LSM6DS_GYRO_RANGE_125_DPS:
      Serial.println("125 degrees/s");
      break;
    case LSM6DS_GYRO_RANGE_250_DPS:
      Serial.println("250 degrees/s");
      break;
    case LSM6DS_GYRO_RANGE_500_DPS:
      Serial.println("500 degrees/s");
      break;
    case LSM6DS_GYRO_RANGE_1000_DPS:
      Serial.println("1000 degrees/s");
      break;
    case LSM6DS_GYRO_RANGE_2000_DPS:
      Serial.println("2000 degrees/s");
      break;
    case ISM330DHCX_GYRO_RANGE_4000_DPS:
      break; // unsupported range for the DSO32
    }

    // dso32.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
    Serial.print("Accelerometer data rate set to: ");
    switch (dso32.getAccelDataRate()) {
    case LSM6DS_RATE_SHUTDOWN:
      Serial.println("0 Hz");
      break;
    case LSM6DS_RATE_12_5_HZ:
      Serial.println("12.5 Hz");
      break;
    case LSM6DS_RATE_26_HZ:
      Serial.println("26 Hz");
      break;
    case LSM6DS_RATE_52_HZ:
      Serial.println("52 Hz");
      break;
    case LSM6DS_RATE_104_HZ:
      Serial.println("104 Hz");
      break;
    case LSM6DS_RATE_208_HZ:
      Serial.println("208 Hz");
      break;
    case LSM6DS_RATE_416_HZ:
      Serial.println("416 Hz");
      break;
    case LSM6DS_RATE_833_HZ:
      Serial.println("833 Hz");
      break;
    case LSM6DS_RATE_1_66K_HZ:
      Serial.println("1.66 KHz");
      break;
    case LSM6DS_RATE_3_33K_HZ:
      Serial.println("3.33 KHz");
      break;
    case LSM6DS_RATE_6_66K_HZ:
      Serial.println("6.66 KHz");
      break;
    }

    // dso32.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
    Serial.print("Gyro data rate set to: ");
    switch (dso32.getGyroDataRate()) {
    case LSM6DS_RATE_SHUTDOWN:
      Serial.println("0 Hz");
      break;
    case LSM6DS_RATE_12_5_HZ:
      Serial.println("12.5 Hz");
      break;
    case LSM6DS_RATE_26_HZ:
      Serial.println("26 Hz");
      break;
    case LSM6DS_RATE_52_HZ:
      Serial.println("52 Hz");
      break;
    case LSM6DS_RATE_104_HZ:
      Serial.println("104 Hz");
      break;
    case LSM6DS_RATE_208_HZ:
      Serial.println("208 Hz");
      break;
    case LSM6DS_RATE_416_HZ:
      Serial.println("416 Hz");
      break;
    case LSM6DS_RATE_833_HZ:
      Serial.println("833 Hz");
      break;
    case LSM6DS_RATE_1_66K_HZ:
      Serial.println("1.66 KHz");
      break;
    case LSM6DS_RATE_3_33K_HZ:
      Serial.println("3.33 KHz");
      break;
    case LSM6DS_RATE_6_66K_HZ:
      Serial.println("6.66 KHz");
      break;
    }

    */


  debugrespond("Actuator test");
  digitalWrite(RelayLR, HIGH);
  delay(1000);
  digitalWrite(RelayLR, LOW);
  delay(1000);
  digitalWrite(RelayLF, HIGH);
  delay(1000);
  digitalWrite(RelayLF, LOW);
  delay(1000);



  
  debugrespond("Setup complete");
    
  }

// ------- Setup Function End -------



// ------- Loop/Main Start -------
  bool breakout;
  int reason; // it switches to several other loops.
  // 1 is room nav
  // 2 is debug looping
  // Anything else does nothing, and maybe logs an error message for now.
  // After the reason is taken care of, break is reset, and the main business comes

 
  void loop() {
  
   while (breakout == false) { // Run the user navigation
    
    if(debugresponse){ Serial.println("----------------------");}
      UpdateData();
      // Autobrake feature
      if (pressureReadingLeft >= 150 || pressureReadingRight >= 150) { brake(1); } else { brake(0); } 
    
      if(debugdisable()) { 
        breakout = true; 
        reason = 1; // FOR NOW, DEBUG SWITCH SENDS IT TO AUTON
         if(debugresponse){ Serial.println("I am in the main loop!"); }
        }
        

      
    delay(100);

   }
  

  if(reason == 1){
     if(debugresponse){ Serial.println("Begin Navigation"); }

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
  *  (USE mapRSSI() or mapRSSIbetter() to turn these into 0-1 scaled functions for accurate relative strength)
  * 
  * Format for vesc motor telemetry: [motor channel].data.[type of information]
  * Motor channels (2): vescML, vescMR
  * Types of information (4): rpm, inpVoltage, ampHours, tachometerAbs
  * 
  */

    
      bool targetreached = false;
      while(targetreached == false){ 
         const float kdrive = 0.2;

         static int cumRSSIL = 0;
         static int cumRSSIR = 0;
         static int oldcRSSIL;
         static int oldcRSSIR;
         static int driveangle = 0;

          if (rssiL < -10) { cumRSSIL = (cumRSSIL + rssiL) / 2; } else {}
          if (rssiR < -10) { cumRSSIR = (cumRSSIR + rssiR) / 2; } else {}

          driveangle = (driveangle + (cumRSSIR - cumRSSIL) / 2);

          if (distanceUltrasonic >= 50) { setMotorSpeed(-driveangle*kdrive, driveangle*kdrive); } 
          else{ setMotorSpeed(0,0); }

          targetreached = !debugdisable();
        delay(100); 
        
        }
      debugrespond("Done with auton!");

  }



    else if(reason == 2) { // Debugging
      debugrespond("Debug mode!");
      debugrespond("");
      
      while(debugdisable() == 1){

        
        getRSSIL();
        getRSSIF();
        getRSSIR();
         static int cumRSSIL = 0;
         static int cumRSSIR = 0;
         static int oldcRSSIL;
         static int oldcRSSIR;
         static int driveangle = 0;

          if (rssiL < -10) { cumRSSIL = (cumRSSIL + rssiL) / 2; } else {}
          if (rssiR < -10) { cumRSSIR = (cumRSSIR + rssiR) / 2; } else {}

          driveangle = (driveangle + (cumRSSIR - cumRSSIL) / 2) / 2;

          Serial.print(cumRSSIL);
          Serial.print(",");
          Serial.println(cumRSSIR);
     
        delay(200);
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


   Serial.println("----------------------");
   Serial.println("");  Serial.println("");  
  }

// ------- Loop/Main End -------
