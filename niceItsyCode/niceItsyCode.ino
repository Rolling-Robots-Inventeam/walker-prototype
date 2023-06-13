#include <string.h>
#include <bluefruit.h>
#include <SoftwareSerial.h>
#include <stdlib.h>
#include <SPI.h>

#define VERBOSE_OUTPUT (0)    // Set this to 1 for verbose adv packet output to the serial monitor
#define ARRAY_SIZE     (2)    // The number of RSSI values to store and compare
#define TIMEOUT_MS     (2500) // Number of milliseconds before a record is invalidated in the list

#if (ARRAY_SIZE <= 1)
  #error "ARRAY_SIZE must be at least 2"
#endif
#if (ENABLE_TFT) && (ENABLE_OLED)
  #error "ENABLE_TFT and ENABLE_OLED can not both be set at the same time"
#endif

// Custom UUID used to differentiate this device.
// Use any online UUID generator to generate a valid UUID.
// Note that the byte order is reversed ... CUSTOM_UUID
// below corresponds to the follow value:
// df67ff1a-718f-11e7-8cf7-a6006ad3dba0
// const uint8_t CUSTOM_UUID[] =
// {
//     0xA0, 0xDB, 0xD3, 0x6A, 0x00, 0xA6, 0xF7, 0x8C,
//     0xE7, 0x11, 0x8F, 0x71, 0x1A, 0xFF, 0x67, 0xDF
// }; 

// 426c7565-4368-6172-6d42-6561636f6e73
const uint8_t CUSTOM_UUID[] =
{
    0x73, 0x6E, 0x6F, 0x63, 0x61, 0x65, 0x42, 0x6D,
    0x72, 0x61, 0x68, 0x43, 0x65, 0x75, 0x6C, 0x42
}; 

//// BLE Beacon
//const uint8_t CUSTOM_MAC[] =
//{
//    0xD6, 0xDD, 0x06, 0x02, 0x34, 0xDD
//}; 

// BLE Beacon EF
const uint8_t CUSTOM_MAC[] =
{
    0xE4, 0xE1, 0x06, 0x02, 0x34, 0xDD
}; 


//
////// BLE Beacon
//const uint8_t CUSTOM_MAC[] =
//{
//   0x6B, 0xE2, 0x06, 0x02, 0x34, 0xDD
//}; 



//// BLE Beacon (wearable)
// const uint8_t CUSTOM_MAC[] =
// {
//    0x7B, 0x9B, 0xFE, 0xEC, 0x60, 0x02
// }; 



//// Mr. Leo's Phone
//const uint8_t CUSTOM_MAC[] =
//{
//    0x2E, 0x18, 0x4D, 0xF0, 0xD0, 0xF7
//}; 

// Mr. Leo's Computer
// uint8_t CUSTOM_MAC[] =
// {
//     0x68, 0x47, 0x3E, 0x4E, 0xE6, 0x7D
// }; 

BLEUuid uuid = BLEUuid(CUSTOM_UUID);
BLEUuid ble_uuid = BLEUuid(CUSTOM_UUID);

boolean return_of_the_mac = false;
boolean missing_mac = true;

// Analog
//int ANALOG0 = 14;
//int ANALOG1 = 15;
//int ANALOG2 = 16;
//int ANALOG3 = 17;
//int ANALOG4 = 18;
//int ANALOG5 = 19;

// Digital
//int DIGITAL2 = 2;
//int DIGITAL5 = 5;
//int DIGITAL7 = 7;
//int DIGITAL9 = 9;
//int DIGITAL10 = 10;
//int DIGITAL11 = 11;
int MicroCtrlMaster_rx = 12; //rx is purple
int MicroCtrlMaster_tx = 13; //tx is white

SoftwareSerial MicroCtrlMaster(MicroCtrlMaster_rx,MicroCtrlMaster_tx);
// Arduino Pin 12 goes to MicroCtrlMaster TX
// Arduino Pin 13 goes to MicroCtrlMaster RX

/* This struct is used to track detected nodes */
typedef struct node_record_s
{
  uint8_t  addr[6];    // Six byte device address
  int8_t   rssi;       // RSSI value
  uint32_t timestamp;  // Timestamp for invalidation purposes
  int8_t   reserved;   // Padding for word alignment
} node_record_t;

node_record_t records[ARRAY_SIZE];

String pad3( int input ){

  if ( input < 10) { 
        return "00" + String(input);
      } 
  if ( input < 100) { 
        return "0" + String(input);
      } 
  return String(input); 

}


void sendRSSITelem(int rssi_telem){
  String cmd;  // initialize


  cmd = rssi_telem;


  // Adafruit hates writing strings over serial, so here we handconvert string into char array
  char charcommand[cmd.length()];
  
  for (int i = 0; i < cmd.length(); i++){
    charcommand[i] = cmd.charAt(i);
  }
  MicroCtrlMaster.write(charcommand);
  MicroCtrlMaster.println(" ");

  Serial.print("Command is: ");
  Serial.println(charcommand); //DEBUG
}


void setup()
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  MicroCtrlMaster.begin(9600);
  
  Serial.println("Bluefruit52 Central Proximity Example");
  Serial.println("-------------------------------------\n");


  /* Clear the results list */
  memset(records, 0, sizeof(records));
  for (uint8_t i = 0; i<ARRAY_SIZE; i++)
  {
    // Set all RSSI values to lowest value for comparison purposes,
    // since 0 would be higher than any valid RSSI value
    records[i].rssi = -128;
  }

  /* Enable both peripheral and central modes */
  if ( !Bluefruit.begin(1, 1) )
  {
    Serial.println("Unable to init Bluefruit");
    while(1)
    {
      digitalToggle(LED_RED);
      delay(100);
    }
  }
  else
  {
    Serial.println("Bluefruit initialized (central mode)");
  }
  
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values

  /* Set the LED interval for blinky pattern on BLUE LED */
  Bluefruit.setConnLedInterval(250);

  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Filter out packet with a min rssi
   * - Interval = 100 ms, window = 50 ms
   * - Use active scan (used to retrieve the optional scan response adv packet)
   * - Start(0) = will scan forever since no timeout is given
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.filterRssi(-100);            // Only invoke callback for devices with RSSI >= -80 dBm
  Bluefruit.Scanner.setInterval(160, 80);       // in units of 0.625 ms
  Bluefruit.Scanner.useActiveScan(true);        // Request scan response data
  Bluefruit.Scanner.start(0);                   // 0 = Don't stop scanning after n seconds
  Serial.println("Scanning ...");
}

/* This callback handler is fired every time a valid advertising packet is detected */
void scan_callback(ble_gap_evt_adv_report_t* report)
{
  node_record_t record;
  
  /* Prepare the record to try to insert it into the existing record list */
  memcpy(record.addr, report->peer_addr.addr, 6); /* Copy the 6-byte device ADDR */
  record.rssi = report->rssi;                     /* Copy the RSSI value */
  record.timestamp = millis();                    /* Set the timestamp (approximate) */

  return_of_the_mac = false;
  /* Attempt to insert the record into the list */
//  if (insertRecord(&record) == 1)                 /* Returns 1 if the list was updated */
  if (true)
  {
    if (memcmp(CUSTOM_MAC, report->peer_addr.addr, 6)==0)
    {
      //DEBUG
      // Serial.printBuffer(report->peer_addr.addr, 6);
      // Serial.println("");
      // Serial.print("RETURN OF THE MAC");
      // Serial.println("");
      return_of_the_mac = true;
      missing_mac = false;
    } else {
      return_of_the_mac = false;
    }
    
  }




/* Fully parse and display the advertising packet to the Serial Monitor
 * if verbose/debug output is requested */
if (VERBOSE_OUTPUT | return_of_the_mac | missing_mac){
  uint8_t len = 0;
  uint8_t buffer[32];
  memset(buffer, 0, sizeof(buffer));

  /* Display the timestamp and device address */
  // if (report->type.scan_response) //DEBUG
  // {
  //   Serial.printf("[SR%10d] Packet received from ", millis());
  // }
  // else //DEBUG
  // {
  //   Serial.printf("[ADV%9d] Packet received from ", millis());

  // }
  // MAC is in little endian --> print reverse
  // Serial.printBufferReverse(report->peer_addr.addr, 6, ':');
  // Serial.println("");

  /* RSSI value */
  // Serial.printf("%14s %d dBm\n", "MY RSSI", report->rssi); //DEBUG
  Serial.printf("%d \n", report->rssi);

  
  // if (missing_mac == false & return_of_the_mac == true){
  if (return_of_the_mac == true){
     sendRSSITelem(report->rssi);
    
    }
    
  // if (report->rssi > -35 & missing_mac== true){
  //   Serial.println("");
  //     Serial.print("THE NEW MAC DADDY");
  //     Serial.println("");
  //     Serial.print(report->rssi);
  //     Serial.println("");
  //     Serial.printBufferReverse(report->peer_addr.addr, 6, ':');
  // Serial.println("");
  
  // memcpy(CUSTOM_MAC, report->peer_addr.addr, 6);
  // Serial.printBufferReverse(CUSTOM_MAC, 6, ':');
  // Serial.println("");
  //     missing_mac = false;
  //   }

  // Serial.println(); //DEBUG
}

  // For Softdevice v6: after received a report, scanner will be paused
  // We need to call Scanner resume() to continue scanning
  Bluefruit.Scanner.resume();
}

int rota = 0;

void loop() 
{
  /* Toggle red LED every second */
  // digitalToggle(LED_RED);
//  Telem(10);

  // rota += 10;
  // Serial.printf("rotation: %d", rota);
 
  delay(500);
}
