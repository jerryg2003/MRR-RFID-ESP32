//Emulates MERG Concentrator for RFID communication with JMRI over WiFi
//REQUIRED MODIFICATION TO MFRC522.cpp when using ESP32 board manager 2.0.8: 
//   change F("x") when occurring in return statements to ((__FlashStringHelper *) "x")
//   OR upgrade to 2.0.9

/* Code provided "AS IS" with no warranties! PLEASE DO NOT DISTRIBUTE FURTHER WITHOUT PERMISSION OF AUTHOR
   Distributed subject to license as specified in the Github reposoitory
   Code on Github may be modified or withdrawn at any time

   Based on sketch from Thomas Seitz in jmriusers @ groups.io  04-2023  (minor modifications for ESP32)
   Copyright (c) 2023 Jerry Grochow
*/

#define VERSION "*** ESP32RFID 2023-05-04 2240"         //Printed during setup
#define VERSIONNUM "-v02-"
#ifdef ESP32
const char compileDate[] = __DATE__ " " __TIME__;
#define LED_BUILTIN 2
#endif

//DEBUG: set to true for debugging information
#define DEBUG true
int long loopCnt = -1;

// Import Libraries
#ifdef ESP32
#include <WiFi.h>                              //Library for ESP32
#else
#incldue <ESP8266WiFi>
#endif
//#include <WiFiMulti.h>                         //If you want to allow searching multiple network SSIDs
#include <SPI.h>                               // SPI library for communicating with the MFRC522 reader
#include <MFRC522.h>                           // MFRC522 library for reading RFID cards

// Wi-Fi credentials
const char* ssid     = "ssid";
const char* password = "password";
const int   port     = 12099;                  //Make sure JMRI RFID connection set to this port
bool   clientPresent = false;                  //Is JMRI connected?

// Define the SS (Slave Select) and RST (Reset) pins for each reader
//*** For reasons not yet understood, ESP32 sensitive to order of the SS pins; some orders will result 
//    in only some RFID cards being found.  05-04-2023 ***
const uint8_t SS_Pins[]  = {17, 16,  5};       //Cannot use ESP32 "output only" pins
const uint8_t RST_Pins[] = {15, 15, 15};       //Cannot use pin which is builtin LED (2 for most ESP32s)
const uint8_t numPossibleReaders = sizeof(SS_Pins) / sizeof(SS_Pins[0]);
//const int numPossibleReaders = 1;            //For testing
const char    readerID[] = {'A', 'B', 'C'};    //MUST BE A-H or I-P for MERG concentrator (coordinate with JMRI setup)
uint8_t       numReaders = 0;                  //Actually found after self-test
uint8_t       numCardsFound = 0;               //For FUTURE logging function

struct RFIDReader {
  char id;
  uint8_t ssPin;
  uint8_t rstPin;
  bool isConnected;
  MFRC522 mfrc522;
  byte nuid[7];

  RFIDReader() : id(0), ssPin(0), rstPin(0), mfrc522(MFRC522(0, 0)) {}
};

//Create objects
WiFiServer server(port);          //Make sure JMRI RFID connection is expecting this port
WiFiClient client;
RFIDReader readers[numPossibleReaders];

//=====================================================================================================
//======================================= SETUP =======================================================
void setup() {

  //Extra power for RFID reader
  pinMode(4, OUTPUT); digitalWrite(4, HIGH);

  //Initialize serial port(s)
  Serial.begin(115200);
  delay(1000);
  Serial.println ("");
  //=================================+++++++ VERSION NUMBER ++++++++++++++++++=========================
  Serial.println(VERSION);                                                                   //  ======
  Serial.print(VERSIONNUM);                                                                  //  ======
#ifdef ESP32
  Serial.print(" Compiled on: ");                                                            //  ======
  Serial.println(compileDate);                                                               //  ======
  Serial.println("");
  Serial.print("** setup() running on core ");
  Serial.println(xPortGetCoreID());
#endif
  //===================================================================================================

  //Start WiFii
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(""); Serial.print("Connected to "); Serial.println(ssid); Serial.print("IP address: "); Serial.println(WiFi.localIP());

  //Start the wifi server
  server.begin();

  //Initialize RFID readers
  numReaders = InitiateRFIDReaders();
  if (numReaders == 0) Serial.println("*** NO READERS FOUND ***");

}  //END setup


//=====================================================================================================
//======================================= LOOP ========================================================
void loop() {
  delay(100);         //SLOW DOWN FOR DEBUGGING ONLY
  loopCnt = loopCnt + 1;
  //  long int loopStartTime = millis();

  if (DEBUG && (loopCnt % 1000 == 0))    {             //Periodically put heartbeat to serial output
    Serial.print("DEBUG: " + String(millis()) + " " + String(loopCnt) + ": ");
    Serial.print("Client.connected: ");  Serial.println(client.connected());
  }

  //Check if JMRI available
  clientPresent = ClientConnected();

  //Check reader whether JMRI connected or not
  numCardsFound = 0;
  for (uint8_t i = 0; i < numPossibleReaders; i++) {
    if (DEBUG && (loopCnt % 100 == 0))    {            //Periodically put heartbeat to serial output
      Serial.print("Checking reader ");   Serial.println(readerID[i]);
    }

    if (ReadRFIDSend(i)) numCardsFound += 1;          //For FUTURE logging function

  }  //END cycle thru readers

}  //END loop
