//Read RFID cards via MFRC522 or PN532 or PN5180 Readers
//Use MQTT communication with JMRI via MQTT Broker over WiFi
//OR Use JMRI RFID Connection over WiFi

//*** NOTE: If using MQTT RFID Connection:  MUST START JMRI *AFTER* THIS SKETCH IS ALREADY RUNNING

// ***Version History at bottom of file

#define VERSION "*** ESP32RFID 2023-12-28 0015"         //Version info printed during setup
#define VERSIONNUM "  -v10a-"

/* *** Using MQTT Connection to JMRI (uncomment #define JMRIMQTT below):
         Set up reporter, sensor, and memory for each reader (my current panel v52f or later)
         JMRI will automatically create IDTags when it see them
         Need to coordinate with JMRI MQTTMemory.jy script (v51 or later)
         Start MQTT broker on same network; start MDNS broadcast "_mqtt" name on same network (or directly input IP address of broker)
   *** FOR MQTT Communication with JMRI:
       Define Reporters in JMRI with system name format "[MQTTClientID (see below)]/[ReaderID]"
       Define Sensors in JMRI with system name format "[MQTTClientID/[ReaderID|0]"
       For example, Reporter: MRE32-E32-RFID01/A and Sensor: MSE32-RFID01/A  and Memory: M.E32-RFID01/A (for specific reader)
       Also:        Sensors:  MSE32-E32-RFID01/ACK (acknowledge receipt of activate Light)
                    Sensors:  MSE32-E32-RFID01/HB  (for receipt of heartbeat)
                    Lights:   MLE32-E32-RFID01/0 (to activate the concentrator)
                    Memory:   M.E32-RFID01/IP (for IP address of concentrator)
                              M,E32-RFID01/L0 (for last tag read on reader L)
                              M.Eew-RFID01/L1 (for loop time on reader L)

   *** Using RFID Connection to JMRI (uncomment #define JMRIRFID below):
         JMRI will automatically set up reporters and sensors and create IDTags as it sees IDTags
   *** FOR RFID Connection in JMRI: select 'MERG Concentrator' and put in jmriRfidport # (Reporters and sensors set up automatically)

*/

/* Code provided "AS IS" with no warranties! PLEASE DO NOT DISTRIBUTE FURTHER WITHOUT PERMISSION OF AUTHOR
   Distributed subject to license as specified in the Github reposoitory
   Code on Github may be modified or withdrawn at any time

   Some based on sketch by Thomas Seitz in jmriusers https://github.com/TMRCI-DEV1/RFID-Concentrators/tree/main/MFRC522  2023-05 (modifications for ESP32, WiFi, MDNS, PN5180)
   Copyright (c) 2023 Jerry Grochow
*/


// ****************************************************************************************************
// ******************** SELECTIONS TO MAKE (by commentinig lines) *************************************
// ****************************************************************************************************

// ***SELECT WHICH LAYOUT CONFIG ***
//#define LAYOUT522            //Selections for layout MFRC522s on far curve 2023-12-26
#define TESTCONFIG             //

// ***SELECT RFID Reader type [One type ONLY for now] ***
#define UseRC522
//#define UsePN532
//#define UsePN5180

// ***SELECT JMRI Communication Method: or comment out the following line ***
#define JMRIMQTT                       //If using MQTT connection to JMRI
//#define JMRIRFID                       //If using JMRI RFID connection

// ***TESTING: Set to "ON" to debug without active JMRI ***
const String JMRITEST = "OFF";

// ***DEBUG: set to true for debugging information ***
#define DEBUG  false
#define DEBUGMQTT false                //MQTT debugging info
#define DEBUGCOMM false

// ****************************************************************************************************
// ******************* END SELECTIONS *****************************************************************
// ****************************************************************************************************


// ****************************************************************************************************
// ******************* PRE PROCESSOR COMMANDS *** DO NOT CHANGE ANYTHING HERE  ************************
// ****************************************************************************************************

// *********** Check communications methods ***********************************************************
#if !(defined(JMRIMQTT) || defined(JMRIRFID))
#warning "NO COMM METHOD SELECTED.  OUTPUT TO SERIAL CONSOLE ONLY."
#endif
#if defined(JMRIRFID) & defined(JMRIMQTT)
#error "BOTH RFID AND MQTT Communication selected. Please uncomment only one."
#endif

#if !(defined(LAYOUT522) || defined(TESTCONFIG))
#error "NO CONFIG SELECTED."
#endif
#if defined(LAYOUT522) & defined(TESTCONFIG)
#error "MORE THAN 1 CONFIG selected. Please uncomment only one."
#endif


//Set up DEBUGing print statements:
#if DEBUG
#define DEBUG_ESP32_RFID(msg) Serial.print(msg)
#else
#define DEBUG_ESP32_RFID(msg)
#endif
#if DEBUGMQTT
#define DEBUG_ESP32_MQTT(msg) Serial.print(msg)
#else
#define DEBUG_ESP32_MQTT(msg)
#endif


// ************** Import Libraries **********************************************************
#include <Streaming.h>

// ********  FOR MFRC522  ********************************************
#ifdef UseRC522
//#include <SPI.h>                       // SPI library for communicating with the MFRC522 reader   ????????????????
#include <MFRC522.h>                   // https://github.com/miguelbalboa/rfid
#endif

// ********  FOR PN532  ********************************************
#ifdef UsePN532
#include <Adafruit_PN532.h>
//ARDUINO SPI:
// #define PN532_SCK  13
// #define PN532_MOSI 11
// #define PN532_SS   10
// #define PN532_MISO 12
//ESP32 SPI:
#define PN532_SCK  18
#define PN532_MOSI 23
#define PN532_SS   5
#define PN532_MISO 19
#endif

// ********  FOR PN5180  ********************************************
#ifdef UsePN5180
//Uses my modified version of these libraries (from tuelly or playfulltechnology on git)
#include <PN5180.h>
#include <PN5180ISO15693.h>
#include <PN5180ISO14443.h>
#endif

// ******************** ESP32 or ESP8266 ******************************
#ifdef ESP32
#define LED_BUILTIN 2
#else
#error "Sketch for ESP32"
#endif
#include <WiFi.h>                      //WiFi Library for ESP32
#include <WiFiMulti.h>                 //If you want to allow searching multiple network SSIDs

#ifdef JMRIMQTT
#include <ESPmDNS.h>
#include <MQTT.h>                      //https://github.com/256dpi/arduino-mqtt
#endif


// ****************************************************************************************************
// ******************* CONSTANTS ** DO NOT CHANGE ANYTHING HERE (CHANGE IN ConfigData.h)  *************
// ****************************************************************************************************

// *************** TIMES and OTHER CONSTANTS *************************************************
const unsigned long oneSec          = 1000;
const unsigned long halfSec         = 500;
const unsigned long shortWait       = 12;
const unsigned long veryShortWait   = 6;
const unsigned long oneMin          = 60 * oneSec;
const unsigned long cardDupInterval = oneSec + oneSec;            //Time to assume same card ID means duplicate read go (4 inchdes at 10sMPH)

// *************** Wi-Fi credentials and Other personal information ***************************************
# include "MyRFID-ConfigData.h"

/* FOLLOWING INFORMATION (at least) SHOULD BE IN INCLUDED CONFIG FILE:
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
  const String  readerIDString = "ABC";          //So can find index easily
*/

// ****************************************************************************************************
// ******************* PROGRAM VARIABLES **  DO NOT CHANGE ANYTHING BELOW HERE  *****************************
// ****************************************************************************************************

#ifdef JMRIMQTT
// *************** MQTT constants *************************************************
const String    mqttChannel             = mqttPersonalPrefix + "trains/";        //{Some MQTT libraries requires char]
enum  SS_TYPES  {SS_LT, SS_MEM, SS_SENS, SS_REPORT, SS_REBOOT, SS_HELLO};
const String    mqttPublishTopics[]     = {"jRecv/lt/", "jRecv/mem/", "jRecv/sens/", "jRecv/reporter/", "", "hello/"};
const uint8_t   numSubTopics            = 2;
const String    mqttSubscribeTopics[]   = {"jSend/lt/", "jDisp/mem/", "", "", "jRbot/REBOOT/", ""};
const String    mqttWillTopic           = "jStat/$state/";    //{Some MQTT libraries requires char]
const char      mqttWillMessage[]       = {"BrokerConnLost"}; //{Some MQTT libraries requires char]

const String    JMRI_LT_ACT             = "/0";      //JMRI Light that sends activation (needs leading slash) [CHANGED AS OF PANEL FILE 52b]
const String    JMRI_SENS_ACK           = "/ACK";    //JMRI Sensor to receive acknowledgement (needs leading slash) [CHANGED AS OF PANEL FILE 52b]
const String    JMRI_HEARTBEAT          = "/HB";     //JMRI Sensor to receive heartbeat (needs leading slash) [CHANGED AS OF PANEL FILE 52b]
const String    JMRI_MEM_ESP32_IP       = "/IP";     //JMRI Memory to hold IP address for ease of access (needs leading slash) [CHANGED AS OF PANEL FILE 52b]
const String    JMRI_MEM_ESP32_ERR      = "/ERR";    //JMRI Memory to hold error messages (in case ESP32 not connected to PC)
const uint8_t   ACK_SENS_NUM            = 127;       //Number to use to indicate "ACK" sensor

// ******************** Various MQTT Processing *************************************************************
String           currentMessage;               //To hold incoming message while being processed
String            topicParts[7];               //To hole parsed incomin
unsigned long int    mqttHBTime = millis();    //Time for next JMRI MQTT heartbeat messages
unsigned long int    mqttHBIncr = 5000;        //5 seconds
bool                 mqttHBFlip = false;
bool          mqttBrokerPresent = false;       //Is JMRI connected to MQTT Broker?

// ****************** JMRI COMMUNICATION OBJECT (per block) **********************************************
struct JMRI      {
  uint8_t       JMRILight;                //Light in block n: ready to receive from reader n
  uint8_t   prevJMRILight;                //FUTURE USE
  uint8_t      JMRISensor;                //Linked to reader
  uint8_t  prevJMRISensor;                //FUTURE USE
  String     JMRIReporter;                //Hold tag string
  String prevJMRIReporter;                //FUTURE USE
  String       JMRIMemory[2];                //Only one memory per reader at this point, 2nd for DEBUG
  String   prevJMRIMemory[2];                //FUTURE USE
  //Constructor
  JMRI() : JMRILight(0), prevJMRILight(0), JMRISensor(0), prevJMRISensor(0) {}
};
#endif

// ********************* RFID READER INFO **************************************************************
const uint8_t numPossibleReaders = sizeof(SS_Pins) / sizeof(SS_Pins[0]);
//const int numPossibleReaders   = 1;          //FOR TESTING
uint8_t       numCardsFound      = 0;          //For FUTURE logging function
bool          JMRIReady = false;               //Light=ON sent by JMRI and this set in message parser
bool          jmriClientPresent  = false;      //Is JMRI connected to WiFi for RFID Connection?
String        readerIDString     = "";         //So can find index easily


// ******************** RFID Reader Info ***************************************************************
struct RFIDReader {
  //Reader info
  char          id;                        //Copied: Reader ID
  ISOTypes isoType;                        //Look for card type: 15693 or 14444
  uint8_t    ssPin;                        //Copied from config: System Select for PN532
  uint8_t   nssPin;                        //Copied from config: Not System select for PN5180
  uint8_t  busyPin;                        //Copied from config: Busy (for PN5180)
  uint8_t   rstPin;                        //Copied from config: Reset (for PN5180)
#ifdef UsePN532
  Adafruit_PN532 nfc14443_532;
#endif
#ifdef UsePN5180
  PN5180ISO15693 nfc15693_5180;            //Must specify which protocol each reader uses
  PN5180ISO14443 nfc14443_5180;
#endif
#ifdef UseRC522
  MFRC522 mfrc522;
#endif
  bool     readerConnected;
  unsigned long int loopStartTime;         //For measurement
  //Card info
  byte     nuidHex[10];                    //Card ID as read (may be more)
  uint8_t  numBytesRead;
  uint8_t  numBytesProcessed;              //To max size of nuidHex array
  byte     checksum;
  byte     prevChecksum;                   //For checking if same card
  String   tagIDString;                    //Including checksum
  bool     cardAlreadyProcessed;           //To avoid multiple reads on same card
  bool     cardDataCleared;                //When messages send to JMRI clearing card
  unsigned long int cardFoundTime;         //When initially seen
  //Functions
  bool     InitiateRFIDReader(uint8_t ii); //Initiate a reader
  bool     ReadRFIDSend (uint8_t ri);      //Read a reader and send data
  bool     ReadRFID (uint8_t ri);
  bool     SendCardData (uint8_t ri);       //Process the results of reading a card
  bool     ClearCardData (uint8_t ri);
  void     FormatRFIDData ();              //Convert from hex to char, computer checksum
  //Constructor
#ifdef UsePN532
  RFIDReader() : id(0), isoType(TYPE14443PN532), ssPin(0), busyPin(0), rstPin(0), nfc14443_532(Adafruit_PN532(0, &SPI)), readerConnected(false), cardAlreadyProcessed(false) {} // Initialize tagPresent to false
#endif
#ifdef UsePN5180
  RFIDReader() : id(0), isoType(TYPEUNK), nssPin(0), busyPin(0), rstPin(0), nfc15693_5180(PN5180ISO15693(0, 0, 0, SPI)), nfc14443_5180(PN5180ISO14443(0, 0, 0, SPI)), readerConnected(false), cardAlreadyProcessed(false) {} // Initialize tagPresent to false
#endif
#ifdef UseRC522
  RFIDReader() : id(0), ssPin(0), rstPin(0), mfrc522(MFRC522(0, 0)), readerConnected(false), cardAlreadyProcessed(false) {} // Initialize tagPresent to false
#endif
};

// ****************** Other Global Variables
uint8_t        numActiveCommMethods = 0;           //Keep track of connections
int long                    loopCnt = -1;
unsigned long int mainLoopStartTime = millis();

// ********************** Create objects ***************************************************************
#if defined(UsePN532) || defined(UseRC522)
SPIClass SPI_nfc(VSPI);   //NEW: as recommended by Git forum for ESP32 (needed ONLY for PN532)
#endif
RFIDReader readers[numPossibleReaders];
WiFiMulti  wifimulti;

#ifdef JMRIRFID
WiFiServer wifiserver(jmriRfidPort);     //Make sure JMRI RFID connection is expecting this port
WiFiClient jmrirfidclient;
#endif
#ifdef JMRIMQTT
JMRI       jmri[numPossibleReaders];           //Should be equal to number of readers
WiFiClient brokerclient;
MQTTClient mqttclient(1024);             //Set up for MQTT (needs 1024 byte buffer for ESP32)
#endif


//=====================================================================================================
//======================================= SETUP =======================================================
void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  //Extra power pin for an RFID reader
  //pinMode(4, OUTPUT); digitalWrite(4, HIGH);

  //Initialize serial port(s)
  Serial.begin(115200);
  BlinkBuiltIn(1, 1);                  //Blink light in case not connected to computer (# of blinks, time multiplier)
  Serial.println ("");
  //=================================+++++++ VERSION NUMBER ++++++++++++++++++=========================
  Serial << VERSION << VERSIONNUM << endl;                                                   //  ======
#ifdef ESP32
  Serial.print("    File" );          Serial.println(__FILE__);                              //  ======
  Serial.print("    Compile info: "); Serial.println(__DATE__ " " __TIME__);                 //  ======
  Serial << (F("** setup() running on core ")) <<  xPortGetCoreID() << endl;                 //  ======
#endif
  //===================================================================================================

  //Initialize RFID readers
  BlinkBuiltIn(2, 1);
  if (InitiateRFIDReaders() == 0) {
    Serial.println("*** NO READERS FOUND ***");
    ManualReboot();             // *************  REBOOT IF NO READERS FOUND *****************
  }
  //Allow search of multiple wifi networks
#if defined(JMRIRFID) || defined(JMRIMQTT)
  Serial << (F("** Connecting wifi as ")) << clientName << (F("  MAC: ")) << WiFi.macAddress() << endl;
  BlinkBuiltIn(3, 1);

  if (DEBUG) ScanNetworks();
  wifimulti.addAP(ssid1, password);
  wifimulti.addAP(ssid2, password);
  IsWiFiConnected();                 //...if not, automatic REBOOT
#endif
#if defined(JMRIRFID)
  //Start the wifi server to get RFID Connection from JMRI (IP:port must match)
  BlinkBuiltIn(4, 2);
  wifiserver.begin();
  Serial.print (F("** WiFi Server Started,"));
  //Serial.print (wifiserver.available())
  Serial << (F(" on port: ")) << jmriRfidPort << endl;
  jmriClientPresent = JmriClientConnected();
  if (!jmriClientPresent) Serial.println(F("!! JMRI RFID client NOT connected. **"));
#elif defined(JMRIMQTT)
  //Start MQTT to communicate with brokers
  BlinkBuiltIn(4, 1);
  mqttBrokerPresent = InitializeMQTT();
  if (!mqttBrokerPresent) Serial.println(F("!! MQTT Broker NOT connected. **"));
#else
  Serial.println (F("!! NO EXTERNAL COMMUNICATION SELECTED **"));
#endif
  // Debugging output for Serial Monitor
  if (DEBUG || numActiveCommMethods == 0)    {             //Make sure to print to console if no communication to JMRI
    Serial.print("     Communication methods: ");
#ifdef JMRIRFID
    Serial << (F("JMRI RFID: ")) << jmriClientPresent << endl;
#elif defined(JMRIMQTT)
    Serial  << (F("MQTT Broker: ")) << mqttBrokerPresent << endl;
#else
    Serial.println("None.");
#endif
  }
  Serial.println (F("*** SETUP COMPLETE ***"));
  BlinkBuiltIn(1, 3);

}  //END setup


//=====================================================================================================
//======================================= LOOP ========================================================
void loop() {
  loopCnt += 1;
  mainLoopStartTime = millis();
#ifdef JMRIMQTT
  mqttclient.loop();                         // Keep MQTT client connected and process incoming messages
  mqttBrokerPresent = MqttBrokerConnected(); //Check if MQTT available
  MqttHeartbeat ();                          //Toggle a JMRI Sensor
#endif
#ifdef JMRIRFID
  jmriClientPresent = JmriClientConnected();  //Check if JMRI RFID Connection available
#endif

  if (DEBUG)      {
    if (loopCnt % 1000 == 0)    {             //Periodically put heartbeat to serial output
      Serial.print("DEBUG: time/loopCnt: " + String(millis()) + "/" + String(loopCnt) + ": ");
#ifdef JMRIMQTT
      Serial.print("  brokerclient.connected: ");  Serial.println(brokerclient.connected());
#elif defined(JMRIRFID)
      Serial.print("  jmrirfidclient.connected: ");  Serial.println(jmrirfidclient.connected());
#else
      Serial.println(" No external comm.");
#endif
    }
  }

  //Check reader
  for (uint8_t i = 0; i < numPossibleReaders; i++) {
    readers[i].loopStartTime = millis();    //Start the clock
    if (readers[i].ReadRFIDSend(i)) numCardsFound += 1;          //For FUTURE logging function
    if (loopCnt % 100 == 0)    {            //Periodically put heartbeat amd timing info
#ifdef JMRIMQTT
      jmri[i].JMRIMemory[1] = (unsigned int) (millis() - readers[i].loopStartTime);    //Send additional heartbeat showing loop time
      if (JMRIReady) MQTTSendMemory(i, 1);
#endif
      if (DEBUG) {
        DEBUG_ESP32_RFID(F("DEBUG: Reader checked 100x: time/one loop: "));  DEBUG_ESP32_RFID(millis());
        DEBUG_ESP32_RFID("/"); DEBUG_ESP32_RFID(millis() - readers[i].loopStartTime); DEBUG_ESP32_RFID(" ");
        DEBUG_ESP32_RFID(readers[i].id); DEBUG_ESP32_RFID(readers[i].readerConnected); DEBUG_ESP32_RFID(": cards found: ");
        DEBUG_ESP32_RFID(numCardsFound); DEBUG_ESP32_RFID("\n");
        //  DEBUG_ESP32_RFID("        Free heap: " + String(ESP.getFreeHeap()) + "\n");
      }
      numCardsFound = 0;
    } //END loopCnt

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
  v04e Change InitializeWifi to use MultiSens version          2023-05-27

  Version for PN5180:
  v05  REMOVE RFID option, only MQTT                           2023-06-07
       Add builtin blink on setup
  v05a Provide for multiple PN5180 readers                     2023-06-08
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
*/
