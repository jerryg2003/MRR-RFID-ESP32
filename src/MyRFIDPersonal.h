//(1) Emulates MERG Concentrator for RFID communication with JMRI over WiFi
//(2) Alternative: use MQTT communication with JMRI via MQTT Broker over WiFi

//Fill in the Personal Information below to use this sketch

/* Code provided "AS IS" with no warranties! PLEASE DO NOT DISTRIBUTE FURTHER WITHOUT PERMISSION OF AUTHOR
   Distributed subject to license as specified in the Github reposoitory
   Code on Github may be modified or withdrawn at any time

   Based on sketch from Thomas Seitz in jmriusers https://github.com/TMRCI-DEV1/RFID-Concentrators/tree/main/MFRC522  2023-05 (modifications for ESP32, WiFi, MDNS)
   Copyright (c) 2023 Jerry Grochow
*/

//WiFi
const char*  ssid               = "XXX";        //WiFi network name
const char*  password           = "XXX";        //WiFi network password
const char*  clientName         = "XXX";        // known to wireless network

//JMRI
const unsigned int jmriRfidPort = 12099;        //Make sure JMRI RFID connection set to this port, if used
const String mqttClientID       = "XXX";        //Name assigned to this ESP32 for MQTT, if used {Some MQTT libraries requires char]
const unsigned int mqttPort     = 1883;
const String mqttPersonalPrefix = "XXX";        //Personal prefix applied to mqttChannel for all mqtt topics

//Hardware configuration fro Microprocessor
const uint8_t SS_Pins[]  = {17, 16,  5};       //Cannot use ESP32 "output only" pins
const uint8_t RST_Pins[] = {15, 15, 15};       //Cannot use pin which is builtin LED (2 for most ESP32s)
const char    readerID[] = {'A', 'B', 'C'};    //MUST BE A-H or I-P for MERG concentrator (coordinate with JMRI setup)
const String  readerIDString = "ABC";          //So can find index easily
