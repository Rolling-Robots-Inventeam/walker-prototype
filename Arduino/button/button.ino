#include <SoftwareSerial.h>
#include <String.h>

SoftwareSerial lora(2,3);

SoftwareSerial mySerial (7,8);


int buttonPin = 4;

void setup() {
  // put your setup code here, to run once:
  mySerial.begin(9600); //gprs baud rate
  Serial.begin(9600);   //gprs baud rate
  delay(500);
}

void loop() {
  // put your main code here, to run repeatedly:
  int buttonVal = digitalRead(buttonPin);
  if(buttonVal == HIGH)
  sendData();
  delay(500);
}
/*
void mymessage(){
  mySerial.print("AT+CMGF=1\r"); //sms in text mode
  delay(100);
  mySerial.print("Sent"); 
  delay(100);
  mySerial.print((char)26);
  delay(100);
  mySerial.println();
}
*/

void sendData()
{
  String mymessage; 
  // default RX address is 0
  String summon = "Summon-walker";
  mymessage = mymessage + "AT+SEND=0" + "," + summon.length() + "," + summon + "\r"; 
  Serial.println(mymessage); 
  lora.println(mymessage);
    delay(50);
    //Serial.println("AT+SEND=0,6,Hello!\r");
}
