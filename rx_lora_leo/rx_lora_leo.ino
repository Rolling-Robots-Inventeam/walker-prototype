#include <SoftwareSerial.h>
 
SoftwareSerial lora(2,3);
// Arduino Pin 2 goes to Lora TX
// Arduino Pin 3 goes to Lora RX
String myString; 
String garbage;
//String data; 
void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
lora.begin(115200);
while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
delay(1000); 
Serial.print("AT\r\n");
lora.print("AT\r\n");
delay(1000); 
//lora.print("AT+ADDRESS=0\r\n");   //needs to be unique
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
lora.setTimeout(250); 
}

void navigateWalker() {
  // move the walker a small amount
  int stepSize = 5; // adjust as needed
  //moveForward(stepSize);
  Serial.println("Walker is moving");
}
bool summoned = false;
enum SummonStatus {NOT_SUMMONED, SUMMONED};
SummonStatus summonStatus = NOT_SUMMONED;

/*
void loop() {
  // put your main code here, to run repeatedly:

if ( lora.available() > 0 )
{
  myString = lora.readString(); 
  Serial.println("Software Serial:");
  Serial.println(myString); 

} else {
delay(100); 
}
}
*/

void loop() {
  // put your main code here, to run repeatedly:
  if (lora.available() > 0 )
  {
    myString = lora.readString(); 
    Serial.println("test" + myString);
    if (myString == "summon") {
      navigateWalker();
    }
    else {
      Serial.println("Walker Stopped");
    }
    Serial.println("Software Serial:");
    Serial.println(myString); 
  } else {
    delay(100); 
  }
}

/*
void loop() {
  // put your main code here, to run repeatedly:
  if (lora.available() > 0 )
  {
    data = lora.readString(); 
    /*if (isDigit(data)) {
        if (data == '1') {
          summoned = false;
          return;
        }
        else {
          summoned = true;
        }
    }
    
    Serial.println("Software Serial:");
    Serial.println(data); 
  } else {
    delay(100); 
  }
}

*/
