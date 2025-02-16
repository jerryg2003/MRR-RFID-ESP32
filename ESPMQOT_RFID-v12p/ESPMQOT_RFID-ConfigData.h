//(1) v10  or later: Emulates MERG Concentrator for RFID communication with JMRI over WiFi
//(2) v05  or later: use MQTT communication with JMRI via MQTT Broker over WiFi
//(3) v09b or later: uses PN532 or PN5180 or MFRC522 RFID Readers; uses ISO14443 tag (also ISO15693 for PN5180)
//(4) vnnn or later: can enter MQTT broker IP into Serial Monitor

//Fill in the Personal Information below to use this sketch

/* Code provided "AS IS" with no warranties! PLEASE DO NOT DISTRIBUTE FURTHER WITHOUT PERMISSION OF AUTHOR
   Distributed subject to license as specified in the Github reposoitory
   Code on Github may be modified or withdrawn at any time

   Based on sketch from Thomas Seitz in jmriusers https://github.com/TMRCI-DEV1/RFID-Concentrators/tree/main/MFRC522  2023-05 (modifications for ESP32, WiFi, MDNS)
   Copyright (c) 2023,2024 Jerry Grochow
*/

// *** WiFi ***
const char*           password  = "your wifi password";          //WiFi network password
#define OTAUID XXX    // ** #define NOT WORKING: UID for OTA in web page definition
#define OTAPW YYY    // ** #define NOT WORKING: Password for OTA in web page definition
#ifdef LAYOUTRFID01
const char*             ssid[]  = {"S1","S2","S3"};             //WiFi network names for this configuration
const char*         clientName  = "ESP32-RFID01-Trains";  // known to wireless network
const char*            OTAhost  = "ESP32-RFID01-Trains-OTA";
#elif defined(TESTCONFIG2)
const char*             ssid[]  = {"S1","S2", ""};          //WiFi network names for this configuration
const char*         clientName  = "ESP32-RFID02-Trains";  // known to wireless network
const char*            OTAhost  = "ESP32-RFID02-Trains-OTA";
#elif defined(LAYOUTRFID03)
const char*              ssid[] = {"S1","S2",""};             //WiFi network names for this configuration
const char*         clientName  = "ESP32-RFID03-Trains";  // known to wireless network
const char*            OTAhost  = "ESP32-RFID03-Trains-OTA";
#elif defined(LAYOUTRFID08)
const char*             ssid[]  = {"S1","",""};           //WiFi network names for this configuration
const char*         clientName  = "ESP32-RFID08-Trains";  // known to wireless network
const char*            OTAhost  = "ESP32-RFID08-Trains-OTA";
#endif

// *** JMRI RFID ***
const unsigned int jmriRfidPort = 12099;                  //Make sure JMRI RFID connection, if used, and this are set the same

// *** MQTT ***
const uint8_t       numBrokerIP = 5;
//Possible MQTT Broker IP addresses, if none found via MDNS search ([0] reserved for MDNS search results
IPAddress    mqttBrokerIP[numBrokerIP] = {IPAddress(0, 0, 0, 0), IPAddress(192, 168, 0, 111), IPAddress(192, 168, 0, 121), IPAddress(10, 0, 0, 131), IPAddress(192, 168, 0, 141)};
const unsigned int     mqttPort = 1883;
const String mqttPersonalPrefix = "XXX";                   //Personal prefix applied to mqttChannel for all mqtt topics


// *** Hardware configuration for Microprocessor ***
//Define the NSS (Not Slave Select), BUSY, and RST (Reset) pins for each PN5180 reader
//Define the SS (Slave Select) pins for each PN532
//Define the SS (Slave Select) and RST (Reset) pins for each MFRC522


//Leave blanks if necessary to keep arrays in sync
#if defined(LAYOUTRFID01)                 //2023-12-26: Two RC522s on layout SS=5,16 RST=15
//Reader ID must coordinate with JMRI
const String       mqttClientID = "E32-RFID01";           //Name assigned to this ESP32 for MQTT, if used {Some MQTT libraries requires char]
const char             RDR_ID[] = {'L', 'K', 'J'};        //MUST BE B-H or I-P for MERG concentrator (coordinate with JMRI setup)
const ISOTypes       ISO_Type[] = {TYPE14443RC522,TYPE14443RC522,TYPE14443RC522};     //Must specify which type of tag to set up for
const uint8_t        SS_Pins[]  = { 5, 16, 17};           //for RC522 on Layout
const uint8_t       RST_Pins[]  = {15, 15, 15};           //for RC522 on layout (can use same pin for multiple readers)

#elif defined(TESTCONFIG2)
//Reader ID must coordinate with JMRI
const String       mqttClientID = "E32-RFID02";           //Name assigned to this ESP32 for MQTT, if used {Some MQTT libraries requires char]
const char             RDR_ID[] = {'A','B'};              //MUST BE B-H or I-P for MERG concentrator (coordinate with JMRI setup)
const ISOTypes       ISO_Type[] = {TYPE14443RC522};       //Must specify which type of tag to set up for
const uint8_t        SS_Pins[]  = {17};                   //for PN532,RC522 (cannot use ESP32 "output only" pins)
//const uint8_t       IRQ_Pins[]  = {4,2};                  //Experiment: PN532 interrupt driven
//const uint8_t       NSS_Pins[]  = {16};                   //for PN5180 (cannot use ESP32 "output only" pins)
//const uint8_t       BUSY_Pins[] = {5};                    //for PN5180
const uint8_t       RST_Pins[]  = {15};                   //for PN5180,RC522 (cannot use pin which is builtin LED (2 for most ESP32s)

#elif defined(LAYOUTRFID03)              //2024-05-20 Three PN532s on layout, far curve at 2'oclock, upper-siding-lower
//Reader ID must coordinate with JMRI
const String       mqttClientID = "E32-RFID03";           //Name assigned to this ESP32 for MQTT, if used {Some MQTT libraries requires char]
const char             RDR_ID[] = {'D', 'E', 'F'};        //MUST BE B-H or I-P for MERG concentrator (coordinate with JMRI setup)
const ISOTypes       ISO_Type[] = {TYPE14443PN532,TYPE14443PN532,TYPE14443PN532};       //Must specify which type of tag to set up for
const uint8_t        SS_Pins[]  = {5,17,16};              //for PN532 on layout (cannot use ESP32 "output only" pins)
const uint8_t       IRQ_Pins[]  = {15,15,15};             //Experiment: PN532 interrupt driven

#elif defined(LAYOUTRFID08)
//Reader ID must coordinate with JMRI
const String       mqttClientID = "E32-RFID08";           //Name assigned to this ESP32 for MQTT, if used {Some MQTT libraries requires char]
const char             RDR_ID[] = {'H', 'G'};             //MUST BE B-H or I-P for MERG concentrator (coordinate with JMRI setup)
const ISOTypes       ISO_Type[] = {TYPE14443PN5180,TYPE14443PN5180}; //Must specify which type of tag to set up for
// *************** MCU SENSITIVE TO ORDER OF BOARD INITIATION AND PIN PLACEMENT **************
const uint8_t       NSS_Pins[]  = {4,16};                 //for PN5180 (cannot use ESP32 "output only" pins)
const uint8_t       BUSY_Pins[] = {22,5};                 //for PN5180 (can use same pin)
const uint8_t       RST_Pins[]  = {15,17};                //for PN5180,RC522 (cannot use pin which is builtin LED (2 for most ESP32s)

#else
#error Layout Config ID
#endif


// *** For reasons not yet understood, ESP32 sensitive to order of the SS pins when multiple RFID readers;
//     some orders will result in only some RFID tags being found.  05-04-2023 ***
