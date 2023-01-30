/* Watch program
 * 
*/

#include <SoftwareSerial.h>
 
SoftwareSerial lora(2,3);
// Arduino Pin 2 goes to Lora TX
// Arduino Pin 3 goes to Lora RX

// LORA TX variables
int vresistor = A1; 
int vrdata = 0; 
int data_length; 
String vresistordata;

bool isSummoningWalker = true;
double bleDistance = 20; // Distance from walker to watch using bluetooth beacon
double bleClosenessThreshold = 1; // Goal distance between walker and watch

void setup() {
  // Intialize LORA transmitter
  Serial.begin(115200);
  lora.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  delay(1000); 
  pinMode(vresistor,INPUT); 
  Serial.print("AT\r\n");
  lora.print("AT\r\n");
  delay(1000); 
  //lora.print("AT+ADDRESS=1\r\n");   //needs to be unique
  //delay(1000);   //wait for module to respond
  lora.print("AT+NETWORKID=6\r\n");   //needs to be same for receiver and transmitter
  delay(1000);   //wait for module to respond
  lora.print("AT+PARAMETER=7,7,1,7\r\n");   //needs to be same for receiver and transmitter
  delay(1000);   //wait for module to respond

  Serial.print("AT+IPR=9600\r\n");
  lora.print("AT+IPR=9600\r\n");
  delay(1000);
  Serial.begin(9600);
  lora.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  delay(1000); 

}

void loop() {
  // After the button has been pressed, begin summoning walker
  if (isSummoningWalker) {
    while (bleDistance > bleClosenessThreshold) { // While walker is still far away from user
      
      // Continue to transmit summon signal
      // readresistor();
      String summonSignal = "summon";
      data_length = summonSignal.length();
      send_data(summonSignal, data_length);
      bleDistance = bleDistance - 1;
      delay(1000); 
      Serial.println(bleDistance);

      // vrdata = vrdata + 1; 
      // if (vrdata >= 100) {
      //   vrdata = 0;
      // }
    }
    isSummoningWalker = false; // Stop summoning walker
    String empty = "";
    send_data(empty, empty.length()); // Send empty string if walker is 
  }
}

void summonWalker() {
  isSummoningWalker = true;
}

void send_data(String dataToSend, int valuelength) {

  String mymessage; 
  // default RX address is 0
  mymessage = mymessage + "AT+SEND=0" + "," + valuelength + "," + dataToSend + "\r"; 
  Serial.println("Sending: " + mymessage); 
  lora.println(mymessage);
  delay(50);
  //Serial.println("AT+SEND=0,6,Hello!\r");
}
