//(1) Emulates MERG Concentrator for RFID communication with JMRI over WiFi
//(2) Alternative: use MQTT communication with JMRI via MQTT Broker over WiFi

/* *** If ESP Board manager 2.0.8: REQUIRED MODIFICATION TO MFRC522.cpp:
        change F("x") when occurring in return statements to ((__FlashStringHelper *) "x")
       If ESP Board manager 2.0.9: No change necessary
   *** If using MQTT Connection to JMRI (uncomment #define JMRIMQTT below): 
         Set up reporter, sensor, and memory for each reader
         JMRI will automatically create IDTags when it see them
         Need to coordinate with JMRI MQTTMemory.jy script (v51 or later)
         Start MQTT broker on same network; start MDNS broadcast "_mqtt" name on same network (or directly input IP address of broker)
   *** If using RFID Connection to JMRI (uncomment #define JMRIRFID below):
         JMRI will automatically set up reporters and sensors and create IDTags as it sees IDTags
*/

/* Code provided "AS IS" with no warranties! PLEASE DO NOT DISTRIBUTE FURTHER WITHOUT PERMISSION OF AUTHOR
   Distributed subject to license as specified in the Github reposoitory
   Code on Github may be modified or withdrawn at any time

   Based on sketch from Thomas Seitz in jmriusers https://github.com/TMRCI-DEV1/RFID-Concentrators/tree/main/MFRC522  2023-05 (modifications for ESP32, WiFi, MDNS)
   Copyright (c) 2023 Jerry Grochow
*/

//***Version History at bottom of file

#define VERSION "*** ESP32RFID 2023-05-16 1630"         //Version info printed during setup
#define VERSIONNUM "-v04a-"
#ifdef ESP32
const char compileDate[] = __DATE__ " " __TIME__;
#define LED_BUILTIN 2
#endif


/* ***FOR MQTT Communication with JMRI:
      Define Reporters in JMRI with system name format "[MQTTClientID (see below)]/[ReaderID]"
      Define Sensors in JMRI with system name format "[MQTTClientID/[ReaderID|0]"
      For example, Reporter: MRE32-E32-RFID01/A and Sensor: MSE32-RFID01/A  and Memory: M.E32-RFID01/A (for specific reader)
      Also:        Sensors: MSE32-E32-RFID01/ACK (acknowledge receipt of activate Light)
                   Sensors: MSE32-E32-RFID01/HB  (for receipt of heartbeat)            
                   Lights: MLE32-E32-RFID01/0 (to activate the concentrator)
                   Memory: M.E32-RFID01/IP (for IP address of concentrator)

   ***FOR RFID Connection in JMRI: select 'MERG Concentrator' and put in jmriRfidport # (Reporters and sensors set up automatically)
*/

//***SELECT JMRI Communication Method: comment or uncomment one of the following lines:
//#define JMRIMQTT                    //If using MQTT connection to JMRI
//#define JMRIRFID                    //If using RFID connection to JMRI

//DEBUG: set to true for debugging information
#define DEBUG false
#define DEBUGM false                //MQTT debugging info

//************** Import Libraries **********************************************************
#ifdef ESP32
#include <WiFi.h>                      //Library for ESP32
#elif defined (ESP8266)
#incldue <ESP8266WiFi>
#else
#error "Sketch for ESP32 or ESP8266"
#endif
#ifdef JMRIMQTT
#include <ESPmDNS.h>
#include <MQTT.h>                      //https://github.com/256dpi/arduino-mqtt
#endif
//#include <WiFiMulti.h>               //If you want to allow searching multiple network SSIDs
#include <SPI.h>                       // SPI library for communicating with the MFRC522 reader
#include <MFRC522.h>                   // https://github.com/miguelbalboa/rfid

//*************** TIMES and OTHER CONSTANTS *************************************************
const unsigned long oneSec         = 1000;
const unsigned long halfSec        = 500;
const unsigned long shortWait      = 12;
const unsigned long veryShortWait  = 6;
const unsigned long oneMin         = 60 * oneSec;

//*************** Wi-Fi credentials *************************************************
const char* ssid         = "ssid";
const char* password     = "password";
const int   jmriRfidPort = 12099;                              //Make sure JMRI RFID connection (if used) set to this port
const char* clientName   = "ESP32-RFID01-JXTrains";            // known to wireless network

#ifdef JMRIMQTT
//*************** MQTT credentials *************************************************
uint8_t     mqttBrokerNum   = 0;                               //Keep track of which broker being used
//If broker found, put it in IP[0];  else try other IP addresses
IPAddress   mqttBrokerIP[]  = {IPAddress(0, 0, 0, 0), IPAddress(10, 0, 0, 3), IPAddress(192, 168, 0, 121), IPAddress(10, 0, 1, 15), IPAddress(192, 168, 0, 143)};
uint8_t numBrokerIP = 5;
const String    mqttClientID            = "E32-RFID01";       //Name assigned to this ESP32 for MQTT {Some MQTT libraries requires char]
const uint      mqttPort                = 1883;
const String    mqttChannel             = "jxtrains/";        //{Some MQTT libraries requires char]
enum  SS_TYPES  {SS_LT, SS_MEM, SS_SENS, SS_REPORT};
const String    mqttPublishTopics[]     = {"Jrecv/lt/", "Jrecv/mem/", "Jrecv/sens/", "Jrecv/reporter/"};
const uint8_t   numSubTopics            = 2;
const String    mqttSubscribeTopics[]   = {"Jsend/lt/", "Jdisp/mem/"};
const String    mqttWillTopic           = "Jstat/$state/";    //{Some MQTT libraries requires char]
const char      mqttWillMessage[]       = {"BrokerConnLost"}; //{Some MQTT libraries requires char]

const String    JMRI_LT_ACT             = "/0";      //JMRI Light that sends activation (needs leading slash) [CHANGED AS OF PANEL FILE 52b]
const String    JMRI_SENS_ACK           = "/ACK";    //JMRI Sensor to receive acknowledgement (needs leading slash) [CHANGED AS OF PANEL FILE 52b]
const String    JMRI_HEARTBEAT          = "/HB";     //JMRI Sensor to receive heartbeat (needs leading slash) [CHANGED AS OF PANEL FILE 52b]
const String    JMRI_MEM_ESP32_IP       = "/IP";     //JMRI Memory to hold IP address for ease of access (needs leading slash) [CHANGED AS OF PANEL FILE 52b]

//******************** Various MQTT Processing *************************************************************
String           currentMessage;               //To hold incoming message while being processed
String            topicParts[7];               //To hole parsed incomin
unsigned long int    mqttHBTime = millis();    //Time for next JMRI MQTT heartbeat messages
unsigned long int    mqttHBIncr = 5000;        //5 seconds
bool                 mqttHBFlip = false;
bool                  JMRIReady = false;       //Light=ON sent by JMRI and this set in message parser
bool          mqttBrokerPresent = false;       //Is JMRI connected to MQTT Broker?

//****************** JMRI COMMUNICATION OBJECT (per block) **********************************************
struct JMRI      {
  uint8_t       JMRILight;                //Light in block n: ready to receive from reader n
  uint8_t   prevJMRILight;
  uint8_t      JMRISensor;                //Linked to reader
  uint8_t  prevJMRISensor;
  String     JMRIReporter;                //Hold tag string
  String prevJMRIReporter;
  String       JMRIMemory;                //Only one memory per reader at this point
  String   prevJMRIMemory;
  //Constructor
  JMRI() : JMRILight(0), prevJMRILight(0), JMRISensor(0), prevJMRISensor(0) {}
};
#endif

//********************* RFID READER INFO **************************************************************
//********************* Define the SS (Slave Select) and RST (Reset) pins for each reader *************
//*** For reasons not yet understood, ESP32 sensitive to order of the SS pins; some orders will result
//    in only some RFID cards being found.  05-04-2023 ***
const uint8_t SS_Pins[]  = {17, 16,  5};       //Cannot use ESP32 "output only" pins
const uint8_t RST_Pins[] = {15, 15, 15};       //Cannot use pin which is builtin LED (2 for most ESP32s)
const uint8_t numPossibleReaders = sizeof(SS_Pins) / sizeof(SS_Pins[0]);
//const int numPossibleReaders = 1;              //FOR TESTING
const char    readerID[] = {'A', 'B', 'C'};    //MUST BE A-H or I-P for MERG concentrator (coordinate with JMRI setup)
const String  readerIDString = "ABC";          //So can find index easily
uint8_t       numDetectedReaders = 0;          //Actually found after self-test
uint8_t       numCardsFound = 0;               //For FUTURE logging function
bool          jmriClientPresent = false;       //Is JMRI connected to WiFi for RFID Connection?

//******************** RFID Reader Info ***************************************************************
struct RFIDReader {
  char id;                                 //Reader ID
  uint8_t ssPin;                           //System select
  uint8_t rstPin;                          //Reset
  MFRC522 mfrc522;
  byte    nuidHex[10];                      //Card ID as read (may be more)
  uint8_t numBytesRead;
  uint8_t numBytesProcessed;                //To max size of nuidHex array
  byte    checksum;
  String  tagIDString;                      //Including checksum
  bool    isConnected;
  bool    tagPresent;
  //Functions
  bool    InitiateRFIDReader(uint8_t ii);   //Initiate a reader
  bool    ReadRFIDSend (uint8_t ri);        //Read a reader and send data
  void    FormatRFIDData ();                //Convert from hex to char, computer checksum
  //Constructor
  RFIDReader() : id(0), ssPin(0), rstPin(0), mfrc522(MFRC522(0, 0)), isConnected(false), tagPresent(false) {} // Initialize tagPresent to false
};


//****************** Other Global Variables
uint8_t    numActiveCommMethods = 0;           //Keep track of connections
int long                loopCnt = -1;
unsigned long int loopStartTime = 0;

//********************** Create objects ***************************************************************
RFIDReader readers[numPossibleReaders];
#ifdef JMRIRFID
WiFiServer wifiserver(jmriRfidPort);     //Make sure JMRI RFID connection is expecting this port
WiFiClient jmriclient;
#elif defined(JMRIMQTT)
JMRI jmri[numPossibleReaders];           //Should be equal to number of readers
WiFiClient brokerclient;
MQTTClient mqttclient(1024);             //Set up for MQTT (needs 1024 byte buffer for ESP32)
#endif


//=====================================================================================================
//======================================= SETUP =======================================================
void setup() {

  //Extra power pin for an RFID reader
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
  Serial.println("");                                                                        //  ======
  Serial.print("** setup() running on core ");                                               //  ======
  Serial.println(xPortGetCoreID());                                                          //  ======
#endif
  //===================================================================================================

#if defined(JMRIRFID) || defined(JMRIMQTT)
//Connect to WiFii
  Serial.print("** Connecting to Wifi as ");  Serial.print(clientName);  Serial.print("  MAC: "); Serial.println(WiFi.macAddress());
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(""); 
  Serial.print("** Connected to ");  Serial.print(ssid);  Serial.print("  on IP address: ");  Serial.println(WiFi.localIP());  
#endif

#ifdef JMRIRFID
  //Start the wifi server to get connection from JMRI
  wifiserver.begin(jmriRfidPort);
  Serial.print("** WiFi Server Started, available: ");
  Serial.println(wifiserver.available());
#elif defined(JMRIMQTT)
  //Start MQTT to communicate with brokers
  InitializeMQTT();
#else
  Serial.println("** NO EXTERNAL COMMUNICATION SELECTED **");
#endif

  //Initialize RFID readers
  numDetectedReaders = InitiateRFIDReaders();
  if (numDetectedReaders == 0) Serial.println("*** NO READERS FOUND ***");

  Serial.println("*** SETUP COMPLETE ***");
}  //END setup


//=====================================================================================================
//======================================= LOOP ========================================================
void loop() {
  //delay(25);         //SLOW DOWN FOR DEBUGGING ONLY
  loopCnt = loopCnt + 1;
  loopStartTime = millis();

#ifdef JMRIMQTT
  //// Keep MQTT client connected and process incoming messages
  mqttclient.loop();
#endif

  if (DEBUG)      {
    if (loopCnt % 1000 == 0)    {             //Periodically put heartbeat to serial output
      Serial.println("DEBUG: " + String(millis()) + " " + String(loopCnt) + ": ");
#ifdef JMRIRFID
      Serial.print("  jmriclient.connected: ");  Serial.println(jmriclient.connected());
#elif defined(JMRIMQTT)
      Serial.print("  brokerclient.connected: ");  Serial.println(brokerclient.connected());
#else
      Serial.println(" No ext comm.");
#endif
    }
  }

  //Check if JMRI available
#ifdef JMRIRFID
  jmriClientPresent = JmriClientConnected();
#elif defined(JMRIMQTT)
  mqttBrokerPresent = MqttBrokerConnected();
#endif

  //Check reader
  numCardsFound = 0;
  for (uint8_t i = 0; i < numPossibleReaders; i++) {
    if (DEBUG)    {
      if (loopCnt % 100 == 0)    {            //Periodically put heartbeat to serial output
        Serial.print("Checked reader 100 times: ");   Serial.print(readerID[i]); Serial.println(readers[i].isConnected ? " Connected" : " NOT Connected");
      }
    }

    if (readers[i].ReadRFIDSend(i)) numCardsFound += 1;          //For FUTURE logging function

  }  //END cycle thru readers

#ifdef JMRIMQTT
  MqttHeartbeat ();                //Toggle a JMRI Sensor
#endif


}  //END loop

// ********************* VERSION HISTORY ************************************************
/*
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
*/
