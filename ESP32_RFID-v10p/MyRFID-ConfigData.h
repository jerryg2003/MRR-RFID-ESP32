//(1) v10 or later: Emulates MERG Concentrator for RFID communication with JMRI over WiFi
//(2) v05 of later: use MQTT communication with JMRI via MQTT Broker over WiFi

//As of v09b:
//Uses PN532 or PN5180 or MFRC522 RFID Readers
//Uses ISO14443 tag (also ISO15693 for PN5180)

//Fill in the Personal Information below to use this sketch

/* Code provided "AS IS" with no warranties! PLEASE DO NOT DISTRIBUTE FURTHER WITHOUT PERMISSION OF AUTHOR
   Distributed subject to license as specified in the Github reposoitory
   Code on Github may be modified or withdrawn at any time

   Based on sketch from Thomas Seitz in jmriusers https://github.com/TMRCI-DEV1/RFID-Concentrators/tree/main/MFRC522  2023-05 (modifications for ESP32, WiFi, MDNS)
   Copyright (c) 2023 Jerry Grochow
*/

//WiFi
const char*  ssid1              = "YourSSID";                     //WiFi network name
const char*  ssid2              = "SecondSSID";                     //Allow search of multiple networks
const char*  password           = "YourPassword";          //WiFi network password
#ifdef LAYOUT522
const char*  clientName         = "ESP32-RFID01-Trains";  // known to wireless network
#elif defined(TESTCONFIG)
const char*  clientName         = "ESP32-RFID02-Trains";  // known to wireless network
#endif

//JMRI RFID
const unsigned int jmriRfidPort = 12099;                    //Make sure JMRI RFID connection set to this port, if used

//MQTT
const uint8_t       numBrokerIP = 5;
//Possible MQTT Broker IP addresses, if none found via MDNS search ([0] reserved for MDNS search results
IPAddress    mqttBrokerIP[numBrokerIP] = {IPAddress(0, 0, 0, 0), IPAddress(192, 168, 0, 111), IPAddress(192, 168, 0, 222), IPAddress(10, 0, 0, 111), IPAddress(10, 0, 0, 222)};

#ifdef LAYOUT522
const String       mqttClientID = "E32-RFID01";           //Name assigned to this ESP32 for MQTT, if used {Some MQTT libraries requires char]
#elif defined(TESTCONFIG)
const String       mqttClientID = "E32-RFID02";           //Name assigned to this ESP32 for MQTT, if used {Some MQTT libraries requires char]
#endif
const unsigned int     mqttPort = 1883;
const String mqttPersonalPrefix = "YourMQTTPrefix";                   //Personal prefix applied to mqttChannel for all mqtt topics

enum  ISOTypes {TYPEUNK, TYPE15693PN5180, TYPE14443PN5180, TYPE14443PN532, TYPE14443RC522};
// ** AS OF V09b: can only have only one type of reader at once.  Can have either type of tags for PN5180. **
const String ISOTypesNames[] = {"Unknown", "ISO15693/PN5180", "ISO14443/PN5180", "ISO14443/PN532", "ISO14443/RC522"};
//Reader ID must coordinate with JMRI
const char             RDR_ID[] = {'J', 'K', 'L', 'M', 'N', 'P'};              //MUST BE A-H or I-P for MERG concentrator (coordinate with JMRI setup)

//Hardware configuration for Microprocessor
//Define the NSS (Not Slave Select), BUSY, and RST (Reset) pins for each PN5180 reader *************
//Define the SS (Slave Select) for each PN532
//Leave blanks if necessary to keep arrays in sync
//2023-12-26: Two RC522s on layout SS=5,16 RST=15
#ifdef TESTCONFIG
const ISOTypes       ISO_Type[] = {TYPE14443PN532};        //Must specify which type of card to set up for
//const ISOTypes       ISO_Type[] = {TYPE14443PN5180};       //Must specify which type of card to set up for
const uint8_t        SS_Pins[]  = {5};                     //for PN532,RC522 (cannot use ESP32 "output only" pins)
const uint8_t       NSS_Pins[]  = {16};                    //for PN5180 (cannot use ESP32 "output only" pins)
const uint8_t       BUSY_Pins[] = {5};                     //for PN5180
const uint8_t       RST_Pins[]  = {17};                    //for PN5180,RC522 (cannot use pin which is builtin LED (2 for most ESP32s)
#elif defined(LAYOUT522)
const ISOTypes       ISO_Type[] = {TYPE14443RC522,TYPE14443RC522};        //Must specify which type of card to set up for
const uint8_t        SS_Pins[]  = {5, 16};                 //for RC522 on Layout
const uint8_t       RST_Pins[]  = {15, 15};                //for RC522 on layout
#endif


// *** For reasons not yet understood, ESP32 sensitive to order of the SS pins when multiple RFID readers;
//     some orders will result in only some RFID cards being found.  05-04-2023 ***
