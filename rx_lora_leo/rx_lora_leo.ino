#include <SoftwareSerial.h>
 
SoftwareSerial lora(2,3);
// Arduino Pin 2 goes to Lora TX
// Arduino Pin 3 goes to Lora RX
String myString; 
String garbage;
char data; 
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
