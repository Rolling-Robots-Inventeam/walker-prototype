#include <SoftwareSerial.h>
 
SoftwareSerial lora(2,3);
// Arduino Pin 2 goes to Lora TX
// Arduino Pin 3 goes to Lora RX
int vresistor = A1; 
int vrdata = 0; 
int data_length; 
String vresistordata;
void setup() {
  // put your setup code here, to run once:

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
  // put your main code here, to run repeatedly:
readresistor();
send_data(vrdata , data_length);
vrdata = vrdata + 1; 
if (vrdata >= 100) {
  vrdata = 0;
}
delay(100); 

}

void readresistor()
{
 //vrdata = analogRead(vresistor);


// find the length of data
 vresistordata = String(vrdata);
 data_length = vresistordata.length();
}

void send_data(int sensorvalue, int valuelength)
{

String mymessage; 
// default RX address is 0
mymessage = mymessage + "AT+SEND=0" + "," + valuelength + "," + sensorvalue + "\r"; 
Serial.println(mymessage); 
lora.println(mymessage);
  delay(50);
  //Serial.println("AT+SEND=0,6,Hello!\r");
}
