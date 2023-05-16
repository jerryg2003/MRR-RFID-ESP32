//Functions for ESP32_RFID

//Emulates MERG Concentrator for RFID communication with JMRI over WiFi
//REQUIRED MODIFICATION TO MFRC522.cpp when using ESP32 libraries 2.0.8: change F("x") when occurring in return statements to ((__FlashStringHelper *) "x")

/* Code provided "AS IS" with no warranties! PLEASE DO NOT DISTRIBUTE FURTHER WITHOUT PERMISSION OF AUTHOR
   Distributed subject to license as specified in the Github reposoitory
   Code on Github may be modified or withdrawn at any time

   Based on sketch from Thomas Seitz in jmriusers @ groups.io  04-2023  (minor modifications for ESP32)
   Copyright (c) 2023 Jerry Grochow
*/


// ****************************************************************************************
uint8_t InitiateRFIDReaders ()      {
  //Initialize RFID readers
  uint8_t numDetectedReaders = 0;

  Serial.println("** Initialize Readers");
  Serial.print("    Default MOSI: ");    Serial.print(MOSI);
  Serial.print("  Default MISO: ");  Serial.print(MISO);
  Serial.print("  Default SCK: ");   Serial.println(SCK);

  for (uint8_t ii = 0; ii < numPossibleReaders; ii++) {
    if (readers[ii].InitiateRFIDReader(ii))  numDetectedReaders += 1;
  }  //END numPossibleReaders

  Serial.print("** Number of readers detected: ");
  Serial.print(numDetectedReaders);
  Serial.print((numPossibleReaders == numDetectedReaders) ? " EQUAL " : " *NOT EQUAL* ");
  Serial.print(numPossibleReaders);
  Serial.println(" readers expected");

  // Blink the LED to indicate the number of detected readers
  pinMode(LED_BUILTIN, OUTPUT);
  for (int ii = 0; ii < numDetectedReaders; ii++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(300);
  }

  return (numDetectedReaders);

}  //END InitiateRFIDReaders

//************************************************************************************************
bool RFIDReader::InitiateRFIDReader(uint8_t ii)   {

  ssPin = SS_Pins[ii];
  rstPin = RST_Pins[ii];
  id = readerID[ii];
  //Start Serial Peripheral Interface for this system (chip) select pin
  SPI.begin(SCK, MISO, MOSI, ssPin);
  delay(200);
  mfrc522 = MFRC522(ssPin, rstPin);
  mfrc522.PCD_Init(ssPin, rstPin);
  // Check if the reader is connected
  delay(250);
  if (mfrc522.PCD_PerformSelfTest()) {
    isConnected = true;
    mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);    // Print debugging information
    Serial.print("    Reader ");
    Serial.print(id);
    Serial.print(" detected on SS pin ");
    Serial.println(ssPin);
  }
  else {
    isConnected = false;
  }
  return (isConnected);
}

// **********************************************************************************************
bool RFIDReader::ReadRFIDSend (uint8_t ri)      {

  bool cardFound  = false;

  if (isConnected)    {
    //If connected, then try to read...
    if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
      cardFound = true;
      digitalWrite(LED_BUILTIN, HIGH);             //Indicator...
      if (DEBUG) {
        Serial.print("DEBUG: Tag detected on reader ");
        Serial.println(readerID[ri]);
      }
      //Copy UID bytes into structure
      numBytesRead = mfrc522.uid.size;
      numBytesProcessed = (mfrc522.uid.size > 9 ? 10 : numBytesRead);
      if (DEBUG)  {
        Serial.print(" Number bytes read: "); Serial.print(numBytesRead);
        Serial.print ("  Bytes processed: "); Serial.println(numBytesProcessed);
      }
      for (uint8_t rj = 0; rj < numBytesProcessed; rj++) {
        nuidHex[rj] = mfrc522.uid.uidByte[rj];
      }

      // If the tag is not already present, process it
      if (!tagPresent) {
        tagPresent = true;
        // Format RFID data (also update checksum
        FormatRFIDData();

        // Debugging output for Serial Monitor
        if (DEBUG || numActiveCommMethods == 0)    {             //Make sure to print to console if no communication to JMRI
          Serial.print("Communication methods: "); 
#ifdef JMRIRFID
          Serial.print(jmriClientPresent); 
#elif defined(JMRIMQTT)
          Serial.print(mqttBrokerPresent);
#else
          Serial.print("None.");          
#endif          
          Serial.print(" Tag:");
          Serial.print(" tagIDString: "); Serial.print(tagIDString);
          Serial.print(" checksum: "); Serial.print(checksum < 0x10 ? "0" : "");  Serial.println(checksum, HEX);
        }

        // Send data to the connected client
#ifdef JMRIRFID
        if (!JmriRFIDSend(ri, checksum))   Serial.println("** JMRI RFID connection no longer active: not reported " + tagIDString);
#elif defined(JMRIMQTT)
        if (!JMRIReady)  Serial.println("** JMRI not ready: tag not reported " + tagIDString);
        else  {
          jmri[ri].prevJMRIReporter = jmri[ri].JMRIReporter; jmri[ri].JMRIReporter = tagIDString; //Put tag ID in Reporter
          if (!MQTTSendReporter(ri))   Serial.println("** MQTT broker lost connection: " + tagIDString + " tag not reported");
          jmri[ri].prevJMRISensor = jmri[ri].JMRISensor; jmri[ri].JMRISensor = 1;                     //ACTIVE
          if (!MQTTSendSensor(ri, "")) Serial.println("** JMRI Sensor not set for " + tagIDString);
          jmri[ri].prevJMRIMemory = jmri[ri].JMRIMemory; jmri[ri].JMRIMemory = tagIDString;
          if (!MQTTSendMemory(ri))     Serial.println("** JMRI Memory not set for " + tagIDString);
        }  //END else
#endif

        // Halt card processing and stop encryption
        mfrc522.PICC_HaltA();
        mfrc522.PCD_StopCrypto1();
      }  //END not tag present
    }  //END tag 
    
    else {    // If a tag is no longer detected, set the sensor to INACTIVE and clear the reporter
      if (tagPresent) {
        tagPresent = false;
        digitalWrite(LED_BUILTIN, LOW);             //Indicator
#ifdef JMRIMQTT
        //Light=ON sent by JMRI when ready to receive from RFID readers
        if (!JMRIReady)  Serial.println("** JMRI not ready: no tag not reported " + tagIDString);
        else {
          jmri[ri].JMRIReporter = "";
          if (!MQTTSendReporter(ri))    Serial.println("** MQTT broker lost connection: " + tagIDString + " not tag not reported ");
          jmri[ri].JMRISensor = 0;
          if (!MQTTSendSensor(ri, ""))  Serial.println("** JMRI Sensor not set for " + tagIDString);
          //Do not clear Memory
          //            if (!MQTTSendMemory(ri))      Serial.println("** JMRI Memory not set for " + tagIDString);
        }
#endif
      }   //END tag present
    }   //END else
      
  }  //END isConnected

  return (cardFound);

}  //END ReadRFIDSend


//**************************************************************************************
void RFIDReader::FormatRFIDData() {
  String rfidData = "";

  // Add NUID to the RFID data string
  for (uint8_t j = 0; j < numBytesProcessed; j++) {
    if (nuidHex[j] < 0x10) {
      rfidData += "0";
    }
    rfidData += String(nuidHex[j], HEX);
  }

  // Calculate the checksum from the reader's NUID
  checksum = nuidHex[0];
  for (uint8_t j = 1; j < numBytesProcessed; j++) {
    checksum ^= nuidHex[j];
  }

  // Add checksum to the RFID data string
  if (checksum < 0x10) rfidData += "0";
  rfidData += String(checksum, HEX);

  // Return the formatted RFID data string
  tagIDString = rfidData;
}


// ************** END COMMON FUNCTIONS *************************************************


//**************************************************************************************
//**************************************************************************************
#ifdef JMRIRFID
//**************************************************************************************
//**************************************************************************************
bool JmriClientConnected()   {
  //Verify JMRI connected to wifiserver
  bool cc  = false;
  if (!jmriclient.connected()) {
    WiFiClient newclient = wifiserver.available();
    if (newclient) {
      jmriclient = newclient;
      Serial.print("** New JMRI client connected: ");
      Serial.println(jmriclient);
      cc = true;
      numActiveCommMethods += 1;        //Add to number of attached communication methods
    }
  }
  else cc = true;

  //If was previously connected, but not now, then reduce num attached comm methods
  if (jmriClientPresent && !cc)  numActiveCommMethods -= 1;

  return (cc);
}

//****************************************************************************************
bool JmriRFIDSend (uint8_t ri, byte csum)          {        //JMRI RFID Connection wantns to see HEX

  if (jmriClientPresent)      {
    jmriclient.write(readers[ri].id);
    for (uint8_t rj = 0; rj < 5; rj++) {
      jmriclient.print(readers[ri].nuidHex[rj] < 0x10 ? "0" : "");
      jmriclient.print(readers[ri].nuidHex[rj], HEX);
    }
    jmriclient.print(csum < 0x10 ? "0" : "");
    jmriclient.print(csum, HEX);

    jmriclient.write(0x0D); // CR
    jmriclient.write(0x0A); // LF
    jmriclient.write('>');

    return (true);
  }

  return (false);
}  //END  RFIDSend

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#endif          //END JMRI RFID connection-specific functions
//*********************************************************************************


//*********************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef JMRIMQTT
//**************************************************************************************
bool MqttBrokerConnected()   {
  //Verify microcontroller connected to MQTT broker, JMRI connected to broker, and JMRI Light is ON
  bool mc  = false;
  if (!mqttclient.connected()) {
    MQTTClient newclient(1024);
    bool newClient = newclient.connect(mqttClientID.c_str(), false);
    if (newClient) {
      mqttclient = newclient;
      Serial.println("New MQTT broker connection: 1");
      mc = true;
      numActiveCommMethods += 1;        //Add to number of attached communication methods
    }
  }
  else mc = true;

  //If was previously connected, but not now, then reduce num attached comm methods
  if (mqttBrokerPresent && !mc)  numActiveCommMethods -= 1;

  return (mc);
}

// *******************************************************************************************
void MqttHeartbeat ()          {               //Toggle a sensor in JMRI via MQTT

  if (mqttHBTime > loopStartTime) return;

  if (JMRIReady)         {
    mqttHBTime = loopStartTime + mqttHBIncr;   // Publish a heartbeat to JMRI (if available) every 5 seconds that can be displayed on a panel
    //Serial.println("X--277s: " + String (loopStartTime) + " " + String (loopCnt) + " " + String(millis()));
    String fullTopic = mqttChannel + mqttPublishTopics[SS_SENS] + mqttClientID + JMRI_HEARTBEAT;
    mqttclient.publish(fullTopic, (mqttHBFlip) ? "ACTIVE" : "INACTIVE", false, 0);    //Retained = true; QoS = 0
    mqttHBFlip = !mqttHBFlip;
  }
}


// *******************************************************************************************
bool MQTTSendReporter (uint8_t rNum)          {

  String fullTopic;
  bool mqPub = false;

  fullTopic = mqttChannel + mqttPublishTopics[SS_REPORT] + mqttClientID + "/" + String(readerID[rNum]);
  mqPub = mqttclient.publish(fullTopic.c_str(), jmri[rNum].JMRIReporter);
  delay(veryShortWait);
  if (DEBUGM) Serial.println("S++291: MQTT Sent: " + String(mqPub) + " [" + fullTopic + " " + jmri[rNum].JMRIReporter + "]");
  return (mqPub);

}  //END MQTTSendReporter


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool MQTTSendSensor (uint8_t rNum, String sState)       {    //Send one sensor

  bool mqPub = false;
  String fullTopic;
  String sMsg;

  if (sState == "")  sMsg =  (jmri[rNum].JMRISensor == 0) ? "INACTIVE" : "ACTIVE";
  else               sMsg =   sState;
  if (rNum > numPossibleReaders) fullTopic = mqttChannel + mqttPublishTopics[SS_SENS] + mqttClientID + JMRI_SENS_ACK;
  else                           fullTopic = mqttChannel + mqttPublishTopics[SS_SENS] + mqttClientID + "/" + String(readerID[rNum]);
  mqPub = mqttclient.publish(fullTopic.c_str(), sMsg.c_str());
  delay(veryShortWait);
  if (DEBUGM) Serial.println("B++309s: MQTTSendSensor: " + String(mqPub) + " [" + fullTopic + " " + sMsg + "]");
  return (mqPub);

}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool MQTTSendMemory (uint8_t rNum)    {     //Send one memory

  bool mqPub = false;
  String fullTopic;

  fullTopic = mqttChannel + mqttPublishTopics[SS_MEM] + mqttClientID + "/" + String(readerID[rNum]);
  mqPub = mqttclient.publish(fullTopic.c_str(), jmri[rNum].JMRIMemory);
  delay(veryShortWait);
  if (DEBUGM) Serial.println("B++320s: MQTT Memories: Sent: " + String(mqPub) + " [" + fullTopic + " " + jmri[rNum].JMRIMemory + "]");
  return (mqPub);
}


//**** Following functions for receiving MQTT messages from JMRI **********************
//*********************************************************************************
//Routine called by mqtt onMessage
void MqttProcessRcvdMsg(String & topic, String & payload) {

  digitalWrite(LED_BUILTIN, HIGH);             //Indicator...

  // Expect topic of:
  //           channel/command("Jsend")/objectType("lt")/microcontrollerID/0
  //  FUTURE   channel/command("Jsend")/objectType("lt")/microcontrollerID/readerName
  //  FUTURE   channel/command("Jdisp")/objectType{"mem")/microcontrollerId/readerName/memory number (nnn)
  //  FUTURE   channel/command("Jrbot")/REBOOT                            <-- REBOOT Microcontroller
  //           channel/command("Jstat")/$state
  const char topicDelimiter = '/';
  //  String     topicParts[7];                 //To hold various parts of message: DECLARED WITH GLOBAL DECLARATIONS
  for (uint8_t c = 0; c < 7; c++) {
    topicParts[c] = ""; //Clear out array
  }
  int8_t     topicNumParts = 0;

  String     topicChannel = "";
  String     topicCommand = "";
  String     topicObjectType = "";
  String     topicMCId = "";
  String     topicReaderName = "";
  String     topicObjectNum = "";          //FUTURE
  String     topicMisc = "";               //FUTURE

  //  int8_t     memNum;
  //  int8_t     sensNum;
  int8_t     readerNum;

  //  char       objectType;                   //first char of object types (to use in CASE statement)

  int   topicLen = topic.length() + 1;
  // Prepare the character array (the buffer)
  char  topicCharArray[topicLen];
  // Copy it over
  topic.toCharArray(topicCharArray, topicLen);


  String  errorMsg     = "";

  if (DEBUGM) Serial.println("B++ 587s: onMessage/ProcRcvd: [" + topic + " " + payload + "]");

  //****** PARSE MESSAGE RECEIVED FROM JMRI
  topicNumParts   = TopicParse (topicLen, &topicCharArray[0], topicDelimiter);
  if (DEBUGM)  {
    Serial.print("B++328s: onMessage/TopicParse: " + String(topicNumParts) + ": ");
    for (uint8_t p = 0; p < 8; p++) {
      Serial.print(topicParts[p]);
      Serial.print(";");
    }
    Serial.println("");
  }

  topicChannel     = topicParts[0];
  topicCommand     = topicParts[1];
  topicObjectType  = topicParts[2];   //objectType = topicObjectType.charAt(0);
  topicMCId        = topicParts[3];
  topicReaderName  = topicParts[4];   readerNum = (readerIDString.indexOf(topicReaderName.charAt(0)) < 0) ? 255 : (readerIDString.indexOf(topicReaderName.charAt(0)));
  topicObjectNum   = topicParts[5];
  topicMisc        = topicParts[6];


  //**************** DO WORK BASED ON COMMAND and GROUP *************************
  switch (topicNumParts)  {
    case 0:     // Error
    case 1:     // Error
    case 2:     // Error
      Serial.println("ERROR1-392s Improper format message: " + topic + " " + payload);
      break;
    case 3:     // Possible status message from JMRI
      if (topicCommand == "Jstat" and topicObjectType == "$state") {
        Serial.println("** JMRI $state last will: " + payload);
        jmri[0].JMRISensor = 0;      //Set the response
        JMRIReady = false;
      }
      else if (topicCommand == "Jrbot" and topicObjectType == "REBOOT") {   // *** REBOOT request ***
        ManualReboot();
      }
      else Serial.println("ERROR3-406s: Unknown command: " + topic + " " + payload);
      break;
    case 4:     // ERROR
      Serial.println("ERROR4-409s: Improper format message: " + topic + " " + payload);
      break;
    case 5:     // Expected message
      if (topicCommand == "Jsend" && topicObjectType == "lt") {
        //Lights specify the reader #
        LightRcvd(readerNum, payload);
        if (DEBUGM) Serial.println("B++415s: Rcvd: Light: " + String(readerNum) + " " + payload);
      }
      else Serial.println("ERROR3-417s: Unknown command: " + topic + " " + payload);
      break;
    case 6:
    case 7:
    default:
      Serial.println("ERRORx-422s: Unknown parsing: " + topic + " " + payload);
      break;

  }        //switch on topicNumParts

  digitalWrite(LED_BUILTIN, LOW);            //Indicator...

}

int TopicParse(int numChars, char* charPtr, char delimiter) {

  // declaring temp string to store the curr "word" upto delimiter
  String temp =  "";
  char* iPtr = charPtr;

  int j = 0;         //element of token array
  for (uint8_t i = 0; i < numChars; i++) {
    // If cur char is not del, then append it to the cur "word", otherwise
    // you have completed the word, print it, and start a new word.
    if (*iPtr != delimiter) {
      //      Serial.println("L44: " + String(*internalptr));
      temp += *iPtr;
      iPtr++;
    }
    else {
      topicParts[j] = temp;
      temp = "";
      iPtr++;  //Skip over delimiter
      j = j + 1;
    }
  }
  topicParts[j] = temp;

  return (j + 1);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void LightRcvd (uint8_t lNum, String lState) {       //lNum = JMRI Light number (used for block reset)

  if (lNum > numPossibleReaders)  {                    //Light = [large number] is JMRI Activate switch
    if (lState == "ON")         {
      JMRIReady = true;
      Serial.println ("** JMRI READY Received: " + String(lNum));
      MQTTSendSensor (lNum, "ACTIVE");                 //Send sensor 0 in response
    }
    else if (lState == "OFF")         {
      JMRIReady = false;
      Serial.println ("** JMRI NOT READY Received: " + String(lNum));
      MQTTSendSensor (lNum, "INACTIVE");                 //Send sensor 0 in response
    }
    else  Serial.println("ERROR-477s: Incorrect message for Light: " + String(lNum) + " " + lState);
  }
  else if (lNum >= 0)       {              //Some other JMRI Light command received
    if (lState == "OFF") jmri[lNum].JMRILight = 0;
    else jmri[lNum].JMRILight = 1;
  }
  else Serial.println("ERROR-485s: Invalid lNum " + String(lNum));
  return;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void MemoryRcvd (uint8_t mNum, String mText) {       //mNUm = Memory number

  if (mNum < 2)  {
    jmri[mNum].JMRIMemory = mText;
  }
  else Serial.println("ERROR1282: Invalid mNum received: " + String(mNum));
  return;

}


//*********** Following functions for initializing MQTT ****************************************************
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool InitializeMQTT()      {

  String fullTopic;

  mqttclient.begin(brokerclient);                //link to wifi object
  mqttclient.onMessage(MqttProcessRcvdMsg);      //Set up to processing received messages
  mqttclient.setKeepAlive(60);                   //Tell broker to have longer than default (seconds)   TEST
  fullTopic = mqttChannel + mqttWillTopic + mqttClientID;
  mqttclient.setWill(fullTopic.c_str(), mqttWillMessage);
  ConnectMQTTBroker();                           //Connect to broker (if not able to connect, will reboot ESP32
  while (!mqttclient.connected()) {}             //WAIT for connect to MQTT broker (shsould always be connected at this point)
  numActiveCommMethods += 1;                     //Increase number of active communication methods
  mqttHBTime = millis();                         //Set up
  if (!SubscribeMQTT()) Serial.println ("***Problem initializing retained MQTT messages");

  return (true);
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool ConnectMQTTBroker()                  {  //Check if MQTT connected

  // Use MDNS to find MQTT Broker IP
  Serial.print("** Start MDNS ");
  for (uint8_t w = 0; w < 10; w++) {
    if (MDNS.begin(clientName)) break;
    Serial.print (".");
  }
  Serial.println("");
  if (!MDNS.begin(clientName)) {
    Serial.println("***   ERROR setting up MDNS responder!");
    //return (false);                    ** Continue on anyway
  }

  Serial.println("** Connecting to MQTT");
  mqttBrokerIP[0] = SelectMQTTBroker(mqttBrokerNum);          //Get perhaps new IP address of broker

  if (DEBUGM) {
    Serial.print("T::CMB519s: Checking if connected: " + String(millis()) + "   ");
    Serial.println(mqttBrokerIP[0]);
  }

  for (uint8_t lp = 0; lp < 9; lp++) {
    if (mqttclient.connect(mqttClientID.c_str())) {
      Serial.print("** MQTT connected to: ");
      Serial.print(mqttBrokerIP[0]);      //Current broker address always stored in IP[0]
      Serial.println(":" + String(mqttPort));
      break;
    }
    else  {                             //Not yet connected to broker
      mqttBrokerNum = (mqttBrokerNum + 1) % numBrokerIP;              //Advance broker try by one
      Serial.print ("    Attempting connection to broker on: ");
      Serial.println(mqttBrokerIP[mqttBrokerNum]);
      mqttBrokerIP[0] = SelectMQTTBroker(mqttBrokerNum);    //Get perhaps new IP address of broker
    }
  } //end for

  if (!mqttclient.connected()) {       //*** REBOOT REBOOT ***//
    //    server.handleClient();       //FUTURE: Check if Web OTA Update before rebooting
    ManualReboot();
  }

  return (true);
}


//+++++++++++++++++++++++++++ Look for MQTT Broker +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
IPAddress SelectMQTTBroker(uint8_t bNum)                   {  //Locate IP and Port

  IPAddress tryMQTTBroker;

  if (DEBUGM) Serial.println("T::SMB554s: " + String(bNum));

  if (bNum != 0)  {
    tryMQTTBroker = mqttBrokerIP[bNum];
  }
  else      {        //If first time, try looking at MDNS.queryService
    uint8_t n = MDNS.queryService("_mqtt", "_tcp");
    Serial.println("    Looking for MQTT Broker broadcast IP");
    if (n == 0) {               //no broker found
      //      delay (oneSec);
      n = MDNS.queryService("_mqtt", "_tcp");      //try one more time
    }
    switch (n)       {        //See how many brokers found, if any
      case 0:
        Serial.println("     No broadcast.");
        break;
      case 1:    //one broker found
        {
          IPAddress firstIP = MDNS.IP(0);       //Check first IP
          if (firstIP[0] == 0)  {               //Just check first part of IP address
            Serial.println ("*** Broadcast Broker IP = null.");
          }
          else  {
            Serial.print("*** Broker IP broadcast: ");
            Serial.print(MDNS.hostname(0));
            Serial.print(" on IP ");
            Serial.print(firstIP);
            Serial.println("");
            tryMQTTBroker = firstIP;                //RETURN broker found
          }
          break;
        }
      default:                             //more than one found
        Serial.println(String (n) + " brokers found.  Using first found: ");
        for (uint8_t i = 0; i < n; ++i) {
          // Print details for each service found
          Serial.print("  ");
          Serial.print(i + 1);
          Serial.print(": ");
          Serial.print(MDNS.hostname(i));
          Serial.print(" (");
          Serial.print(MDNS.IP(i));
          Serial.print(":");
          Serial.print(MDNS.port(i));
          Serial.println(")");
        }
        tryMQTTBroker = MDNS.IP(0);                //RETURN first IP in list
        break;
    }  //end of switch
  } //end of brokerNum = 0

  //Set up to see if broker found at this address
  mqttclient.setHost(tryMQTTBroker, mqttPort);
  return (tryMQTTBroker); //

}   //end of subroutine


//++++++++++++++++ Do MQTT Subscriptions and initializations +++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool SubscribeMQTT ()                 {

  String fullTopic;
  bool mqPub[4];

  Serial.println("** MQTT Resets and Subscriptions");

  //Send MQTT initization messages to clear out prior retained messages
  fullTopic = mqttChannel + mqttWillTopic; fullTopic.remove(fullTopic.length() - 1,1); //Strip off final slash
  mqPub[0] = mqttclient.publish(fullTopic.c_str(), "", true, 0);   //JMRI state: Retained = true; QoS = 0
  Serial.println("    Reset MQTT object: " + fullTopic);
  fullTopic = mqttChannel + mqttSubscribeTopics[SS_LT] + mqttClientID + JMRI_LT_ACT;
  mqPub[1] = mqttclient.publish(fullTopic.c_str(), "OFF", true, 0);    //Light 0: JMRI set MC ready   ***FOR TESTING WITHOUT JMRI, set to: ON
  Serial.println("    Reset MQTT object: " + fullTopic);
  fullTopic = mqttChannel + mqttPublishTopics[SS_SENS] + mqttClientID + JMRI_SENS_ACK;
  mqPub[2]  = mqttclient.publish(fullTopic.c_str(), "", true, 0);    //Retained = true; QoS = 0
  Serial.println("    Reset MQTT object: " + fullTopic);
  fullTopic = mqttChannel + mqttSubscribeTopics[SS_MEM] + mqttClientID + JMRI_MEM_ESP32_IP;
  mqPub[2]  = mqttclient.publish(fullTopic.c_str(), "", true, 0);    //Memory: Retained = true; QoS = 0
  Serial.println("    Reset MQTT object: " + fullTopic);
  //Put IP address in JMRI memory for easy access
  mqPub[3] = mqttclient.publish(mqttChannel + mqttPublishTopics[SS_MEM] + mqttClientID + JMRI_MEM_ESP32_IP, WiFi.localIP().toString(), true, 0);  //Retaned = true

  //Do required MQTT subscription after RESETs
  fullTopic = mqttChannel + "Jstat/#";     //Status messages from JMRI
  mqttclient.subscribe(fullTopic);
  Serial.println("    Subscribing to " + fullTopic);
  for (uint8_t st = 0; st < 2; st++)    {
    fullTopic = mqttChannel + mqttSubscribeTopics[st] + mqttClientID + "/#";
    Serial.println("    Subscribing to " + fullTopic);
    //bool subscribe(const char topic[], uint8_t qos);
    mqttclient.subscribe(fullTopic);
  }

  return (mqPub[0] && mqPub[1] && mqPub[2] && mqPub[3]);
}

//**************************************************************************************************
void ManualReboot()    {               //CAN BE REQUESTED VIA MQTT MESSAGE OR AS A RESULT OF NOT CONNECTING TO MQTT

  Serial.println("**E**681s: Manual REBOOT request");
  delay (5 * oneSec);
  Serial.println("");
  delay(2 * oneSec);
  ESP.restart();                    //*** REBOOT ESP32 ***
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//******************************************************************************************************
#endif   //END JMRI MQTT-specific functions
//****************************************************************************************************
// *=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
