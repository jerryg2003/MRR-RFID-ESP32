//Read RFID cards via MFRC522 or PN532 or PN5180 Readers
//Use RFID connection or use MQTT connection to JMRI
//Use "Over The Air" update via web page at IP address for loading new sketch

//UPDATES [all listed at bottom of this file]:
//(1) v05  or later: use MQTT communication with JMRI via MQTT Broker over WiFi
//(2) v09b or later: uses PN532 or PN5180 or MFRC522 RFID Readers; uses ISO14443 tag (also ISO15693 for PN5180)
//(3) v10  or later: Emulates MERG Concentrator for RFID communication with JMRI over WiFi
//(4) v10f or later: can enter LAST PART of MQTT broker IP into Serial Monitor
// ***Complete version history at bottom of file


// ***   MAKE SELECTIONS FOLLOWING THESE COMMENTS ***


#define VERSION "*** ESP32 MQTT OTA _RFID 2025-02-14-1135"  //Version info printed during setup
#define VERSIONNUM "-MQOTRF-v12h-"                          //Version number also published via MQTT

//COMMUNICATIONS TO JMRI: Two possible ways to communicate with JMRI, via  "MQTT Connection" and "RFID Connection":
/* *** Using MQTT Connection to JMRI (uncomment #define JMRIMQTT below):
       1. Set up reporter, sensor, and memory for each reader (my current panel v52f or later)
       2. JMRI will automatically create IDTags when it see them
       3. Need to coordinate with JMRI MQTTMemory.jy script (v51 or later)
       4. Start MQTT broker on same network as MC; 
           to "automagically" find broker IP address, start MDNS broadcast of "_mqtt" name on same network; 
           alternatively, directly input IP address of broker in ConfigData or on IDE console
    ** Further instructions for JMRI setup for MQTT communication:
       Define Reporters in JMRI with system name format "[MQTTClientID (see below)]/[ReaderID]"
         EXAMPLE:   Reporter: [MR]E32-E32-RFID01/L and   and Memory:  (for specific reader)
       Define Sensors in JMRI with system name format "[MQTTClientID/[ReaderID|Suffix]"
         EXAMPLE:   Sensors:  [MS]E32-RFID01/ACK (acknowledge receipt of activate Light)
                    Sensors:  [MS]E32-RFID01/HB  (for receipt of heartbeat)
                    Sensor:   [MS]E32-RFID01/L
       Define Lights in JMRI with system name format "[MQTTClientID/[ReaderID|0]"
         EXAMPLE:   Lights:   [ML]E32-RFID01/0 (to activate this concentrator)
       Define Memories in JMRI with system name format "M.[MQTTClientID/[ReaderID]/[Suffix]"
         EXAMPLE:   Memory:   [M]M.E32-RFID01/IP (for IP address of this concentrator)
                              [M]M.E32-RFID01/VERS (for version number of this sketch)
                              [M]M.E32-RFID01/L
                              [M}M,E32-RFID01/L0 (for last tag read on reader L)
                              [M]M.E32-RFID01/L1 (for loop time on reader L)
   *** Using RFID Connection to JMRI (uncomment #define JMRIRFID below):
         JMRI will automatically set up reporters and sensors and create IDTags as it sees IDTags
    ** Furhter instructions for RFID Connection in JMRI: select 'MERG Concentrator' and put in jmriRfidport # (Reporters and sensors set up automatically)
    ** NOTE: If using MQTT RFID Connection:  MUST START JMRI *AFTER* THIS SKETCH IS ALREADY RUNNING

*/

/* Code provided "AS IS" with no warranties! PLEASE DO NOT DISTRIBUTE FURTHER WITHOUT PERMISSION OF AUTHOR
   Distributed subject to license as specified in the Github reposoitory
   Code on Github may be modified or withdrawn at any time

   Some initial code based on sketch by Thomas Seitz in jmriusers https://github.com/TMRCI-DEV1/RFID-Concentrators/tree/main/MFRC522  2023-05
   Copyright (c) 2023, 2024, 2025 Jerry Grochow
*/


// ****************************************************************************************************
// ************* MAKE PROCESSING SELECTIONS (by commenting/uncommenting lines or changing values) ********************************
// ****************************************************************************************************

// * 1 ** SELECT WHICH LAYOUT CONFIG (only uncomment 1 line) ***
//#define LAYOUTRFID01            //Selections for RFID01 layout MFRC522s on 2 o'clock curve 2023-12-26
#define TESTCONFIG2  //Testing MFRC522
//#define LAYOUTRFID03          //Selections for RFID03 layout with PN532s on 10 o'clock curve 2024-05-15
//#define LAYOUTRFID08            //Testing PN5180: RFID08

// * 2 ** SELECT RFID Reader type [One type ONLY for now] ***
#define UseRC522
//#define UsePN532
//#define UsePN5180

// * 3 ** SELECT JMRI Communication Method (uncomment ONE of the following lines) ***
#define JMRIMQTT  //If using MQTT connection to JMRI
//#define JMRIRFID                       //If using JMRI RFID connection

// * 4[Optional) ** TESTING: Set to true to debug without active JMRI ***
#define JMRITEST false

// *** 5(Optional) ** DEBUG: set to true for various debugging information sent to IDE console ***
#define DEBUGRFOT false
#define DEBUGMQTT false  //MQTT debugging info
#define DEBUGCOMM false
#define TIMING false  //Reader timing tests

// ****************************************************************************************************
// ******************* END SELECTIONS *****************************************************************
// ****************************************************************************************************


// ****************************************************************************************************
// ******************* PRE-PROCESSOR COMMANDS *** DO NOT CHANGE ANYTHING HERE  ************************
// ****************************************************************************************************
#ifdef LAYOUTRFID01
#define PCONFIG "LAYOUTRFID01"  //To print
#warning "LAYOUTRFID01"
#elif defined(TESTCONFIG2)
#define PCONFIG "TESTCONFIG2"  //To print
#warning "TESTCONFIG2"
#elif defined(LAYOUTRFID03)
#define PCONFIG "LAYOUTRFID03"  //To print
#warning "LAYOUTRFID03"
#elif defined(LAYOUTRFID08)
#define PCONFIG "LAYOUTRFID08A"  //To print
#warning "LAYOUTRFID08A"
#else
#define PCONFIG "UNKNOWN"
#warning "UNKOWN"
#endif

// *********** Check communications methods ***********************************************************
#if defined(JMRIRFID) & defined(JMRIMQTT)
#error "BOTH RFID AND MQTT Communication selected. Please uncomment only one."
#endif
#ifdef JMRIMQTT
#define VPROT "MQTT"
#elif defined(JMRIRFID)
#define VPROT "RFID"
#else
#warning "NO COMM METHOD SELECTED.  OUTPUT TO SERIAL CONSOLE ONLY."
#endif

#if !(defined(LAYOUTRFID01) || defined(TESTCONFIG2) || defined(LAYOUTRFID03) || defined(LAYOUTRFID08))
#error "NO CONFIG SELECTED."
#endif
#if defined(LAYOUTRFID01) & (defined(TESTCONFIG2) || defined(LAYOUTRFID03) || defined(LAYOUTRFID08))
#error "MORE THAN 1 CONFIG selected. Please uncomment only one."
#elif defined(TESTCONFIG2) & (defined(LAYOUTRFID03) || defined(LAYOUTRFID08))
#error "MORE THAN 1 CONFIG selected. Please uncomment only one."
#elif defined(LAYOUTRFID03) & defined(LAYOUTRFID08)
#error "MORE THAN 1 CONFIG selected. Please uncomment only one."
#endif

//Set up DEBUGing print statements:
#if DEBUGMQTT
#define DEBUG_ESP32_MQTT(msg) Serial.print(msg)
#else
#define DEBUG_ESP32_MQTT(msg)
#endif


// ****************************************************************************************************
// ************** Import Libraries **********************************************************
// ****************************************************************************************************

#include <Streaming.h>
//***OTA Update ***
#include <Update.h>
#include <WebServer.h>
#include "OTAwebpages4.h"         //File that holds the HTML for the web pages (in same directory)
#include "OLEDdisplaygraphics.h"  //File that holds the graphics for the OLED display (in same directory)

// ********  FOR MFRC522  ********************************************
#ifdef UseRC522
#warning "RC522 reader selected."
#include <MFRC522.h>  // https://github.com/miguelbalboa/rfid: MFRC 1-4-11 with mod: *backlen
#endif

// ********  FOR PN532  ********************************************
#ifdef UsePN532
#warning "PN532 reader selected."
#include <Adafruit_PN532.h>  //v.1.3.3 UNMODIFIED
//ARDUINO SPI:
// #define PN532_SCK  13
// #define PN532_MOSI 11
// #define PN532_SS   10
// #define PN532_MISO 12
//ESP32 SPI:
#define PN532_SCK 18
#define PN532_MOSI 23
#define PN532_SS 5
#define PN532_MISO 19
#endif

// ********  FOR PN5180  ********************************************
#ifdef UsePN5180
#warning "PN5180 reader selected."
//Uses my slightly modified version of these libraries (from tuelly or playfulltechnology on git)
#include <PN5180.h>
#include <PN5180ISO15693.h>
#include <PN5180ISO14443.h>
#endif

// ******************** ESP32 or ESP8266 ******************************
#ifdef ESP32
#define LED_BUILTIN 2
#else
#warning "Sketch designed for ESP32"
#endif
#include <WiFi.h>       //WiFi Library for ESP32
#include <WiFiMulti.h>  //If you want to allow searching multiple network SSIDs


// ******************** MQTT *****************************************
#ifdef JMRIMQTT
#include <ESPmDNS.h>
#include <MQTT.h>  //https://github.com/256dpi/arduino-mqtt
#endif


// ****************************************************************************************************
// ******************* CONSTANTS ** DO NOT CHANGE ANYTHING HERE (CHANGE IN ConfigData.h)  *************
// ****************************************************************************************************

// *************** TIMES and OTHER CONSTANTS *************************************************
const unsigned long oneSec = 1000;
const unsigned long halfSec = 500;
const unsigned long shortWait = 12;
const unsigned long veryShortWait = 3;  //Was 6 until v10b (issue with JMRI MQTT setting block value)
const unsigned long oneMin = 60 * oneSec;
const unsigned long fiveMin = 5 * oneMin;
const unsigned long cardDupInterval = oneSec + oneSec;  //Time to assume same card ID means duplicate read go (4 inchdes at 10sMPH)

// *** RFID Reader Types - ** DO NOT CHANGE ** ***
enum ISOTypes { TYPEUNK,
                TYPE15693PN5180,
                TYPE14443PN5180,
                TYPE14443PN532,
                TYPE14443RC522 };
// ** AS OF V09b: can only have only one type of reader at once.  Can have either type of tags for PN5180. **
const String ISOTypesNames[] = { "Unknown", "PN5180/ISO15693", "PN5180/ISO14443", "PN532/ISO14443", "RC522/ISO14443" };

// *************** Wi-Fi credentials and Other personal information ***************************************
#include "ESPMQOT_RFID-ConfigData.h"

/* FOLLOWING INFORMATION (at least) SHOULD BE IN INCLUDED ConfigData FILE:
  //WiFi
  const char*  ssid               = "XXX";       //WiFi network name
  const char*  password           = "XXX";       //WiFi network password
  const char*  clientName         = "XXX";       // known to wireless network
  //JMRI communication
  const unsigned int jmriRfidPort = nnnn;        //Make sure JMRI RFID connection set to this port, if used
  //MQTT
  //Possible MQTT Broker IP addresses, if none found via MDNS search ([0] reserved for MDNS search results
  const uint8_t      numBrokerIP  = n;
  IPAddress    mqttBrokerIP[numBrokerIP] = {IPAddress(0,0,0,0), IPAddress(nnn,nnn,nnn,nnn), IPAddress(nnn,nnn,nnn,nnn), IPAddress(nnn,nnn,nnn,nnn), IPAddress(nnn,nnn,nnn,nnn)};
  const String mqttClientID       = "XXX";       //Name assigned to this ESP32 for MQTT, if used {Some MQTT libraries requires char]
  const unsigned int mqttPort     = nnnn;
  const String mqttPresonalPrefix = "XXX";       //Personal prefix applied to mqttChannel for all mqtt topics
  //Microprocessor pin and reader assignments (for however many readers you have)
  const uint8_t SS_Pins[]  = {nn, nn, nn};       //Cannot use ESP32 "output only" pins
  const uint8_t RST_Pins[] = {nn, nn, nn};       //Cannot use pin which is builtin LED (2 for most ESP32s)
  const char    readerID[] = {'A', 'B', 'C'};    //MUST BE A-H or I-P for MERG concentrator (coordinate with JMRI setup)
*/

// ****************************************************************************************************
// ******************* PROGRAM VARIABLES **  DO NOT CHANGE ANYTHING BELOW HERE  *****************************
// ****************************************************************************************************

#ifdef JMRIMQTT
// *************** MQTT constants *************************************************
const String mqttChannel = mqttPersonalPrefix + "trains/";  //{Some MQTT libraries requires char]
enum SS_TYPES { SS_LT,
                SS_MEM,
                SS_SENS,
                SS_REPORT,
                SS_REBOOT,
                SS_HELLO,
                SS_GOODBYE };
const String mqttPublishTopics[] = { "jRecv/lt/", "jRecv/mem/", "jRecv/sens/", "jRecv/rpt/", "", "hello/", "goodbye/" };
const uint8_t numSubTopics = 7;
const String mqttSubscribeTopics[] = { "jSend/lt/", "jDisp/mem/", "", "", "jRbot/REBOOT/", "","" };
const String JMRIWillTopic = "jStat/$state/";            //{Some MQTT libraries requires char]
const char mqttMcWillMessage[] = { "MC Disconnected" };  //Will message for this microcontroller  {Some MQTT libraries requires char]

const String JMRI_LT_ACT = "/0";             //JMRI Light that sends activation (needs leading slash) [CHANGED AS OF PANEL FILE 52b]
const String JMRI_SENS_ACK = "/ACK";         //JMRI Sensor to receive acknowledgement (needs leading slash) [CHANGED AS OF PANEL FILE 52b]
const String JMRI_HEARTBEAT = "/HB";         //JMRI Sensor to receive heartbeat (needs leading slash) [CHANGED AS OF PANEL FILE 52b]
const String JMRI_MEM_ESP32_IP = "/IP";      //JMRI Memory to hold IP address for ease of access (needs leading slash) [CHANGED AS OF PANEL FILE 52b]
const String JMRI_MEM_ESP32_VERS = "/VERS";  //JMRI Memory to hold version of sketch for ease of access (needs leading slash) [CHANGED AS OF PANEL FILE 60d]
const String JMRI_MEM_ESP32_ERR = "/ERR";    //JMRI Memory to hold error messages (in case ESP32 not connected to PC) [FUTURE]
const uint8_t ACK_SENS_NUM = 127;            //Number to use to indicate "ACK" sensor
const uint8_t ACT_LT_NUM = 0;                //Number received from JMRI to activate MC

// ******************** Various MQTT Processing *************************************************************
String currentMessage;                       //To hold incoming message while being processed
String topicParts[7];                        //To hole parsed incoming
unsigned long mqttHBIncr = 6000;             //6 seconds
unsigned long mqttHBTime = millis();         //Time for next JMRI MQTT heartbeat messages
unsigned long JMRIHBTime = mqttHBTime;       //For checking on when JMRI last alive
unsigned long JMRIHBDuration = 20 * oneMin;  //Reboot if JMRI "inactive" for long period of time
bool mqttHBFlip = false;
bool mqttBrokerPresent = false;  //Is JMRI connected to MQTT Broker?

// ****************** JMRI COMMUNICATION OBJECT (per block) **********************************************
struct JMRI {
  uint8_t JMRILight;         //Light in block n: ready to receive from reader n
  uint8_t prevJMRILight;     //FUTURE USE
  uint8_t JMRISensor;        //Linked to reader
  uint8_t prevJMRISensor;    //FUTURE USE
  String JMRIReporter;       //Hold tag string
  String prevJMRIReporter;   //FUTURE USE
  String JMRIMemory[3];      //0:Tag ID, 1:Timing, 2:ReReads
  String prevJMRIMemory[3];  //FUTURE USE
  //Constructor
  JMRI()
    : JMRILight(0), prevJMRILight(0), JMRISensor(0), prevJMRISensor(0) {}
};
#endif

// ********************* RFID READER INFO **************************************************************
#ifdef UsePN5180
const uint8_t numPossibleReaders = sizeof(NSS_Pins) / sizeof(NSS_Pins[0]);
#else
const uint8_t numPossibleReaders = sizeof(SS_Pins) / sizeof(SS_Pins[0]);
#endif
//const int numPossibleReaders   = 1;          //FOR TESTING
uint8_t numCardsFound = 0;       //For FUTURE logging function
bool JMRIReady = false;          //Light=ON sent by JMRI and this set in message parser
bool jmriClientPresent = false;  //Is JMRI connected to WiFi for RFID Connection?
String readerIDString = "";      //So can find index easily; created in InitializeRFIDReaders


// ******************** RFID Reader Info ***************************************************************
struct RFIDReader {
  //Reader info
  char id;           //Copied: Reader ID
  ISOTypes isoType;  //Look for card type: 15693 or 14444
  uint8_t ssPin;     //Copied from config: System Select for RC522/PN532
  uint8_t nssPin;    //Copied from config: Not System select for PN5180
  uint8_t busyPin;   //Copied from config: Busy (for PN5180)
  uint8_t rstPin;    //Copied from config: Reset (for PN5180)
  uint8_t irqPin;    //EXPERIMENT

#ifdef UsePN532
  Adafruit_PN532 nfc14443_532;
#endif
#ifdef UsePN5180
  PN5180ISO15693 nfc15693_5180;  //Must specify which protocol each reader uses
  PN5180ISO14443 nfc14443_5180;
#endif
#ifdef UseRC522
  MFRC522 mfrc522;
#endif
  bool readerConnected;
  unsigned long int loopStartTime;  //For measurement
  //Card info
  byte nuidHex[10];  //Card ID as read (may be more)
  uint8_t numBytesRead;
  uint8_t numBytesProcessed;  //To max size of nuidHex array
  byte checksum;
  byte prevChecksum;               //For checking if same card
  String tagIDString;              //Including checksum
  bool cardAlreadyProcessed;       //To avoid ple reads on same card
  uint8_t numTimesTagReRead;       //Number times same card read in one interval
  bool cardDataCleared;            //When messages send to JMRI clearing card
  unsigned long cardFoundTime;     //When initially seen
  unsigned long cardLastSeenTime;  //When last seen
  //Functions
  bool InitiateRFIDReader(uint8_t ii);  //Initiate a reader
  bool ReadRFIDSend(uint8_t ri);        //Read a reader and send data
  bool ReadRFID(uint8_t ri);
  bool SendCardData(uint8_t ri);  //Process the results of reading a card
  bool ClearCardData(uint8_t ri);
  void FormatRFIDData();  //Convert from hex to char, computer checksum
  //Constructor
#ifdef UsePN532
  RFIDReader()
    : id(0), isoType(TYPE14443PN532), ssPin(0), busyPin(0), rstPin(0), nfc14443_532(Adafruit_PN532(0, &SPI)), readerConnected(false), cardAlreadyProcessed(false), numTimesTagReRead(0) {}  // Initialize tagPresent to false
#endif
#ifdef UsePN5180
  RFIDReader()
    : id(0), isoType(TYPEUNK), nssPin(0), busyPin(0), rstPin(0), nfc15693_5180(PN5180ISO15693(0, 0, 0, SPI)), nfc14443_5180(PN5180ISO14443(0, 0, 0, SPI)), readerConnected(false), cardAlreadyProcessed(false), numTimesTagReRead(0) {}  // Initialize tagPresent to false
#endif
#ifdef UseRC522
  RFIDReader()
    : id(0), ssPin(0), rstPin(0), mfrc522(MFRC522(0, 0)), readerConnected(false), cardAlreadyProcessed(false), numTimesTagReRead(0) {}  // Initialize tagPresent to false
#endif
};

// ****************** Other Global Variables
uint8_t numActiveCommMethods = 0;  //Keep track of connections
int long loopCnt = -1;
unsigned long int mainLoopStartTime = millis();

// ********************** Create objects ***************************************************************
#if defined(UsePN532) || defined(UseRC522)
SPIClass SPI_nfc(VSPI);  //NEW: as recommended by Git forum for ESP32 (needed ONLY for PN532)
#endif

//USED BY ALL
RFIDReader readers[numPossibleReaders];  //Must be after SPIClass creation
WiFiMulti wifimulti;
WebServer OTAserver(80);     // **WEB SERVER on Port 80 for OTA Web Update
WiFiClient OTAclientobject;  // for OTA Web Update

#ifdef JMRIRFID
WiFiServer jmrirfidwifiserver(jmriRfidPort);  //Make sure JMRI RFID connection is expecting this port
WiFiClient jmrirfidwificlient;
#endif
#ifdef JMRIMQTT
JMRI jmri[numPossibleReaders];  //Should be equal to number of readers
WiFiClient mqttbrokerwificlient;
MQTTClient mqttclient(1024);  //Set up for MQTT (needs 1024 byte buffer for ESP32)
#endif


//=====================================================================================================
//======================================= SETUP =======================================================
//=====================================================================================================
void setup() {

  bool setupOK = true;  //Will be set false if anything fails
  pinMode(LED_BUILTIN, OUTPUT);
  //Extra power pin for an RFID reader
  //pinMode(4, OUTPUT); digitalWrite(4, HIGH);

  //Initialize serial port(s)
  Serial.begin(115200);
  BlinkBuiltIn(1,3);  //C
  BlinkBuiltIn(1,1); 
  BlinkBuiltIn(1,3); 
  BlinkBuiltIn(1,1); 
  BlinkBuiltIn(0,3);  //space
  BlinkBuiltIn(2,3);  //Q
  BlinkBuiltIn(1,1); 
  BlinkBuiltIn(1,3); 
  BlinkBuiltIn(0,4);  //space
  Serial.println("");
  //=================================+++++++ VERSION NUMBER ++++++++++++++++++=========================
  Serial << VERSION << VERSIONNUM << PCONFIG << "-" << VPROT << endl;  //  ======
#ifdef ESP32
  Serial.print("    File");
  Serial.println(__FILE__);  //  ======
  Serial.print("    Compile info: ");
  Serial.println(__DATE__ " " __TIME__);                                     //  ======
  Serial << (F("** setup() running on core ")) << xPortGetCoreID() << endl;  //  ======
#endif
  //===================================================================================================

  //DO THIS FIRST TO ALLOW TIME IF OTA REQUESTED
  Serial << (F("** Connecting wifi as ")) << clientName << (F("  MAC: ")) << WiFi.macAddress() << endl;

  //Allow search of multiple wifi networks
  if (DEBUGCOMM) ScanNetworks();
  wifimulti.addAP(ssid[0], password);
  if (*ssid[1] != *("")) wifimulti.addAP(ssid[1], password);  //If second SSID specified for this config
  if (*ssid[2] != *("")) wifimulti.addAP(ssid[2], password);  //If third SSID specified for this config
  IsWiFiConnected();                                          //...if not, automatic REBOOT
  BlinkBuiltIn(1, 1);                //W
  BlinkBuiltIn(2, 3);
  BlinkBuiltIn(0, 4);  //space
                       //******* Web OTA Setup
  if (WebOTASetup()) {
    Serial.print("OK:  Web OTA Setup on: ");
    Serial.println(OTAhost);
    OTAserver.handleClient();
    BlinkBuiltIn(3,3);
    BlinkBuiltIn(0,4);  //space
  } else {
    Serial.println("** NO OTA Web **");
    setupOK = false;
  }

  //Initialize RFID readers
  uint8_t rdrsInit = InitiateRFIDReaders();
  if (rdrsInit == 0) {
    Serial.println("*!* NO READERS FOUND ***");
    ManualReboot("No RFID readers found");  // *************  REBOOT IF NO READERS FOUND *****************
  }
  BlinkBuiltIn(1, 1);             //R
  BlinkBuiltIn(1, 3);
  BlinkBuiltIn(1, 1);
  BlinkBuiltIn(0, 4);  //space
  BlinkBuiltIn(rdrsInit, 1);
  BlinkBuiltIn(0, 4);  //space
  if (rdrsInit < numPossibleReaders) setupOK = false;


#if defined(JMRIRFID) || defined(JMRIMQTT)
#endif
#if defined(JMRIRFID)
  //Start the wifi server to get RFID Connection from JMRI (IP:port must match)
  jmrirfidwifiserver.begin();
  Serial.print(F("** WiFi Server Started,"));
  BlinkBuiltIn(1, 1);            //J
  BlinkBuiltIn(3, 3);
  BlinkBuiltIn(0, 4);  //space
  //Serial.print (jmrirfidwifiserver.available())
  Serial << (F(" on port: ")) << jmriRfidPort << endl;
  jmriClientPresent = JmriClientConnected();
  if (!jmriClientPresent) {
    Serial.println(F("!! JMRI RFID client NOT connected. **"));
    setupOK = false;
  }
#elif defined(JMRIMQTT)
  //Start MQTT to communicate with brokers
  mqttBrokerPresent = InitializeMQTT();
  if (!mqttBrokerPresent) {
    Serial.println(F("!! MQTT Broker NOT connected. **"));
    setupOK = false;
    BlinkBuiltIn(2,3);  //M
    BlinkBuiltIn(0,4);  //space
  }
#else
  Serial.println(F("!! NO EXTERNAL COMMUNICATION SELECTED **"));
#endif
  // Debugging output for Serial Monitor
  if (DEBUGRFOT || numActiveCommMethods == 0) {  //Make sure to print to console if no communication to JMRI
    Serial.print("     Communication methods: ");
#ifdef JMRIRFID
    Serial << (F("JMRI RFID: ")) << jmriClientPresent << endl;
#elif defined(JMRIMQTT)
    Serial << (F("MQTT Broker: ")) << mqttBrokerPresent << endl;
#else
    Serial.println("None.");
#endif
  }
  Serial.print(setupOK ? "***" : "!!!!");  //Warning if something failed
  Serial.println(F(" SETUP COMPLETE ***"));
  BlinkBuiltIn(5, 1);

}  //END setup


//=====================================================================================================
//======================================= LOOP ========================================================
void loop() {

  mainLoopStartTime = millis();
  loopCnt += 1;

  //For Web OTA Update
  OTAserver.handleClient();

#ifdef JMRIMQTT
  mqttclient.loop();                          // Keep MQTT client connected and process incoming messages
  mqttBrokerPresent = MqttBrokerConnected();  //Check if MQTT available
  MqttHeartbeat();                            //Toggle a JMRI Sensor
  JMRIMqttHeartBeat();                        //Check for JMRI sending over MQTT
#endif
#ifdef JMRIRFID
  jmriClientPresent = JmriClientConnected();  //Check if JMRI RFID Connection available
#endif

  if (DEBUGRFOT) {
    if (loopCnt % 1000 == 0) {  //Periodically put heartbeat to serial output
      Serial.print("==493: time/loopCnt: " + String(millis()) + "/" + String(loopCnt) + ": ");
#ifdef JMRIMQTT
      Serial.print("  mqttbrokerwificlient.connected: ");
      Serial.println(mqttbrokerwificlient.connected());
#elif defined(JMRIRFID)
      Serial.print("  jmrirfidwificlient.connected: ");
      Serial.println(jmrirfidwificlient.connected());
#else
      Serial.println(" No external comm.");
#endif
    }
  }

  //Check reader
  for (uint8_t i = 0; i < numPossibleReaders; i++) {
    readers[i].loopStartTime = millis();                 //Start the clock
    if (readers[i].ReadRFIDSend(i)) numCardsFound += 1;  //For FUTURE logging function
    if (loopCnt % 500 == 0) {                            //Periodically put timing info
#ifdef JMRIMQTT
      jmri[i].JMRIMemory[1] = String(millis() - readers[i].loopStartTime);  //Send additional heartbeat showing loop time
      if (JMRIReady) MQTTSendMemory(i, 1);
#endif
      if (DEBUGRFOT) {
        Serial << (F("==514: Reader checked 100x: time/one loop: ")) << millis() << "/" << millis() - readers[i].loopStartTime << " "
               << readers[i].id << readers[i].readerConnected << ": cards found: " << (numCardsFound) << ("\n");
        //  Serial << ("        Free heap: ") << String(ESP.getFreeHeap()) << endl;
      }
      numCardsFound = 0;
    }  //END loopCnt

  }  //END cycle thru readers

  //delay(25);       // ** MAY NOT WORK ** SLOW DOWN FOR READER TO RESET

}  //END loop


// ********************* VERSION HISTORY ************************************************
/*
  Version for MFRC522:
  v01  RFID Wifi connected to JMRI                             2023-04-20
  v02  Restructure code                                        2023-05-04
  v03  Add MQTT as alternative to RFID Connection to JMRI      2023-05-07
  v03a Various changes via Tom Seitz
       Add JMRI Ready by Reader, Heartbeat for panel display   2023-05-09
  v03c Change names for JMRI objects, fix messages sent        2023-05-11
       Remove JMRI Ready by reader
       Added MDNS.begin
       Added delay in clearing reader
  v03d Code cleanup                                            2023-05-12
  v03e Fix reading of ID tags
  v03f Code cleanup                                            2023-05-13
  v04  Put RFID Reader functions into structure                2023-05-13
  v04a Put MQTT Publish Topics into an array                   2023-05-14
  v04b Create include file for personal info                   2023-05-19
  v04c Change structure of MQTT broker connection startup      2023-05-21
  v04d Change JMRI topic naming from "Jxxxx" to "jXxxx"        2023-05-26
  v04e Change InitializeWifi to use Sens version          2023-05-27

  Version for PN5180:
  v05  REMOVE RFID option, only MQTT                           2023-06-07
       Add builtin blink on setup
  v05a Provide for ple PN5180 readers                     2023-06-08
  v06  Specify which type of ISO tags each reader is set for   2023-06-09
  v06b Add code for ISO14443 processing                        2023-06-12
  v06c Add DEBUG macros                                        2023-06-12
       Continue experimenting with ISO14443 code
  v06d Some code cleanup                                       2023-06-14
  v06e No checksum on ISO14443 card read                       2023-08-16
       Fix numBytesProcessed for ISO144443                     2023-08-16
  v06f Change to PN5180ISO14443::readCardSerial                2023-09-20
       Blink LED on reboot                                     2023-09-25

  Version for PN532 (based on v06f)
  v07  Change for PN532                                        2023-12-13
  v07a Add interval to avoid dup card read                     2023-12-15
       Slowo down loop to allow PN532 to capture all reads     2023-12-15
  v08  Revised how RFID Reader functions work                  2023-12-16
       Removed loop slow down, replaced with wait after read   2023-12-16
       Added hello world HB when JMRI not connected            2023-12-16
  v08a Fixed clearing of RFID data                             2023-12-17
  v08b Change order of MQTT messages                           2023-12-18
       Changed variable names to integrate PN5180 and PN532    2023-12-19

  Version for ESP32 PN532/PN5180/MFRC522 branched from v08b:
  v09   Include selective code for PN532 and PN5180 readers     2023-12-18
  v09a  Change reader memory names for more than one mem        2023-12-19
  v09b  Add MFRC522 init and read                               2023-12-20
  v10   Add RFID Connection method back in                      2023-12-24
        Add DEBUGCOMM for JMRI RFID, make fixes                 2023-12-26
        Fix JMRITEST settings                                   2023-12-26
        Convert UID to upper case for compat with MFRC522       2023-12-27
  v10a  Changed print out on tag processing                     2023-12-27
  v10b  Changed veryShortWait to improve JMRI block value set   2024-01-07
  V10c  Limited numBytesProcessed with PN532 to 5 for compat    2024-01-10
        Settled on MQTT Reporter before Sensor for JMRI         2024-01-10
  v10d  Add reader timing tests                                 2024-01-12
  v10e  Limit number of bytes in UID reported to 5 per JMRI     2024-01-12
  v10f  Add console input for MQTT Broker IP (last segment)     2024-01-26

  Version ESP32_RFID_OTA
  v11   Add OTA                                                 2024-05-26
  v11a  Change wifi object names                                2024-05-29
        Change light flashes on setup                           2024-05-29
  v11b  Change MQTT mem pub to not retained                     2024-07-28
	v12   Stop using Multiwifi and do ssid switch manually        2024-08-13
	v12a  Fix LED blinks and MQTT status messages                 2024-10-30
	      Add check for JMRI HB                                   2024-10-30
  v12b  MODIFY PN532 library per https://github.com/adafruit/Adafruit-PN532/issues/80    
                                                                2024-11-02
        Replace MDNS.IP with MDNS.address                       2024-11-11 
  v12c  Add back wifimulti with conditionals                    2024-11-22 
        USE STANDARD PN532 library AdaFruit PN532 1.3.3         2024-11-23  
        Code cleanup in common.ino                              2024-11-23
        Fix bug in proc <5 char RFID tag                        2024-11-23	
        Add PN532 reset()                                       2024-11-30
  v12d  Some small updates                                      2024-12-10
        Add MQTT pub to reboot                                  2024-12-10
        Fix up code re readerNum in MQTT interpret              2024=12=25
        Added "_" in MDNS query (for reliability?)              2024-12-25
  v12e  Change will processing topics                           2024-12-27
	      Fix some console debug messages                         2024-12-27
				Change JMRITEST to bool                                 2024-12-27
  v12f  Various minor changes                                   2025-01-04
        Changes to mqttinit                                     2025-01-04
	      Publish # times card read                               2025-01-06
  v12g  Update ReadRFIDSend to accommodate "next car"           2025-01-07
	      Change last will topic                                  2025-01-07
	v12h  Change some preprocessor                                2025-01-14
	      Change ACT_LT_NUM to 0                                  2025-01-14
        Change setup blinks to Morse code                       2025-01-23
				Correct use of numSubTopics                             2025-02-05
        Move 522 PCD_setAntennaGain up (no noticable effect)    2025-02-14

*/
