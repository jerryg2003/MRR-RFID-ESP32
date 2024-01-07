//Functions for ESP32_RFID


/* Code provided "AS IS" with no warranties! PLEASE DO NOT DISTRIBUTE FURTHER WITHOUT PERMISSION OF AUTHOR
   Distributed subject to license as specified in the Github reposoitory
   Code on Github may be modified or withdrawn at any time

   Based on sketch from Thomas Seitz in jmriusers @ groups.io  04-2023  (minor modifications for ESP32)
   Copyright (c) 2023 Jerry Grochow
*/

// **************************************************************************************
// **************************************************************************************
#ifdef JMRIRFID
// **************************************************************************************
// **************************************************************************************
bool JmriClientConnected()   {
  //Verify JMRI connected to wifiserver
  bool cc  = false;
  if (!jmrirfidclient.connected()) {
    WiFiClient newrfidclient = wifiserver.available();
    //    cc = jmrirfidclient.connect(WiFi.localIP(), jmriRfidPort, 20);        //TEST if this works instead
    if (DEBUGCOMM) {
      Serial.print(F("DEBUG cc25: ")); Serial.print(wifiserver.hasClient());
    }
    if (newrfidclient) {
      //    if (newrfidclient.connected()) {         //TEST if this works instead
      jmrirfidclient = newrfidclient;
      Serial.print("** JMRI RFID client connected: "); Serial.println(jmrirfidclient.connected());
      cc = true;
      numActiveCommMethods += 1;        //Add to number of attached communication methods
    }
    //else Serial.println(F("** JMRI RFID client NOT connected. **"));
  }
  else cc = true;

  //If was previously connected, but not now, then reduce num attached comm methods
  if (jmriClientPresent && !cc)  numActiveCommMethods -= 1;

  return (cc);
}

// ****************************************************************************************
bool JmriRFIDSend (uint8_t ri)          {        //JMRI RFID Connection wantns to see HEX
  if (DEBUGCOMM) Serial << (F("DEBUG sr44: ")) << jmriClientPresent << readers[ri].id << ": " << readers[ri].tagIDString << " " << readers[ri].checksum << endl;
  if (!jmriClientPresent)  jmriClientPresent = JmriClientConnected();
  if (DEBUGCOMM) Serial << (F("DEBUG sr46: ")) << jmriClientPresent << readers[ri].id << ": " << readers[ri].tagIDString << " " << readers[ri].checksum << endl;
  if (jmriClientPresent)  {
    unsigned int charsWritten = 0;
    readers[ri].numBytesProcessed = 5;           //Limitation of the JMRI software 2023-12-26
    charsWritten += jmrirfidclient.write(readers[ri].id);             //Send ID to JMRI
    for (uint8_t rj = 0; rj < readers[ri].numBytesProcessed; rj++) {
      charsWritten += jmrirfidclient.print(readers[ri].nuidHex[rj] < 0x10 ? "0" : "");
      charsWritten += jmrirfidclient.print(readers[ri].nuidHex[rj], HEX);
    }
    charsWritten += jmrirfidclient.print(readers[ri].checksum < 0x10 ? "0" : "");
    charsWritten += jmrirfidclient.print(readers[ri].checksum, HEX);

    charsWritten += jmrirfidclient.write(0x0D);      // CR
    charsWritten += jmrirfidclient.write(0x0A);      // LF
    charsWritten += jmrirfidclient.write('>');       //Send ">" instead of ETX

    if (DEBUGCOMM) Serial << (F("DEBUG sr59: ")) << charsWritten << endl;
    return (true);
  }

  return (false);
}  //END  RFIDSend

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#endif          //END JMRI RFID connection-specific functions
// *********************************************************************************



// *********************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef JMRIMQTT
// **************************************************************************************
bool MqttBrokerConnected()   {
  //Verify microcontroller connected to MQTT broker, JMRI connected to broker, and JMRI Light is ON
  bool mc  = false;
  if (!mqttclient.connected()) {
    MQTTClient newclient(1024);
    bool newClient = newclient.connect(mqttClientID.c_str(), false);
    if (newClient) {
      mqttclient = newclient;
      Serial.println("** New MQTT broker connection: 1");
      mc = true;
      numActiveCommMethods += 1;        //Add to number of attached communication methods
    }
  }
  else mc = true;

  //If was previously connected, but not now, then reduce num attached comm methods
  if (mqttBrokerPresent && !mc)  numActiveCommMethods = max(numActiveCommMethods - 1, 0);

  return (mc);
}

// *******************************************************************************************
void MqttHeartbeat ()          {               //Toggle a sensor in JMRI via MQTT

  if (mqttHBTime > mainLoopStartTime) return;

  if (JMRIReady)         {
    mqttHBTime = mainLoopStartTime + mqttHBIncr;   // Publish a heartbeat to JMRI (if available) every 5 seconds that can be displayed on a panel
    //Serial.println("X--277s: " + String (loopStartTime) + " " + String (loopCnt) + " " + String(millis()));
    String fullTopic = mqttChannel + mqttPublishTopics[SS_SENS] + mqttClientID + JMRI_HEARTBEAT;
    mqttclient.publish(fullTopic, (mqttHBFlip) ? "ACTIVE" : "INACTIVE", false, 0);    //Retained = true; QoS = 0
    mqttHBFlip = !mqttHBFlip;
  }
  else      {  // Publish a heartbeat message if no JMRI.
    mqttHBTime = mainLoopStartTime + oneMin;
    mqttclient.publish(mqttChannel + mqttPublishTopics[SS_HELLO] + String(mqttClientID), String(loopCnt));
  }

}


// *******************************************************************************************
bool MQTTSendReporter (uint8_t rNum)          {

  String fullTopic;
  bool mqPub = false;

  fullTopic = mqttChannel + mqttPublishTopics[SS_REPORT] + mqttClientID + "/" + String(readers[rNum].id);
  mqPub = mqttclient.publish(fullTopic.c_str(), jmri[rNum].JMRIReporter);
  delay(veryShortWait);
  DEBUG_ESP32_MQTT("S++291: MQTT Sent: " + String(mqPub) + " [" + fullTopic + " " + jmri[rNum].JMRIReporter + "]\n");
  return (mqPub);

}  //END MQTTSendReporter


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool MQTTSendSensor (uint8_t rNum, String sState)       {    //Send one sensor

  bool mqPub = false;
  String fullTopic;
  String sMsg;

  if (sState == "")  sMsg =  (jmri[rNum].JMRISensor == 0) ? "INACTIVE" : "ACTIVE";
  else               sMsg =   sState;
  if (rNum == ACK_SENS_NUM) fullTopic = mqttChannel + mqttPublishTopics[SS_SENS] + mqttClientID + JMRI_SENS_ACK;
  else                      fullTopic = mqttChannel + mqttPublishTopics[SS_SENS] + mqttClientID + "/" + String(readers[rNum].id);
  mqPub = mqttclient.publish(fullTopic.c_str(), sMsg.c_str());
  delay(veryShortWait);
  DEBUG_ESP32_MQTT("B++309s: MQTTSendSensor: " + String(mqPub) + " [" + fullTopic + " " + sMsg + "]\n");
  return (mqPub);

}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool MQTTSendMemory (uint8_t rNum, uint8_t mNum)    {     //Send one memory

  bool mqPub = false;
  String fullTopic;

  //Append memory # to JMRI memory name for multiple memories
  fullTopic = mqttChannel + mqttPublishTopics[SS_MEM] + mqttClientID + "/" + String(readers[rNum].id) + String(mNum);
  mqPub = mqttclient.publish(fullTopic.c_str(), jmri[rNum].JMRIMemory[mNum]);
  delay(veryShortWait);
  DEBUG_ESP32_MQTT("B++320s: MQTT Memories: Sent: " + String(mqPub) + " [" + fullTopic + " " + jmri[rNum].JMRIMemory[mNum] + "]\n");
  return (mqPub);
}


// **** Following functions for receiving MQTT messages from JMRI **********************
// *********************************************************************************
//Routine called by mqtt onMessage
void MqttProcessRcvdMsg(String & topic, String & payload) {

  digitalWrite(LED_BUILTIN, HIGH);             //Indicator...

  // Expect topic of:
  //           channel/command("jSend")/objectType("lt")/microcontrollerID/0                 [To tell MC to activate]
  //           channel/command("jSend")/objectType("lt")/microcontrollerID/readerName        [To tell MC to re-init a reader]
  //  FUTURE   channel/command("jDisp")/objectType{"mem")/microcontrollerId/readerName/memory number (nnn)
  //           channel/command("jRbot")/REBOOT                            <-- REBOOT Microcontroller
  //           channel/command("jStat")/$state
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
  int8_t     readerNum;                    //-1 signals error

  //  char       objectType;                   //first char of object types (to use in CASE statement)

  int   topicLen = topic.length() + 1;
  // Prepare the character array (the buffer)
  char  topicCharArray[topicLen];
  // Copy it over
  topic.toCharArray(topicCharArray, topicLen);

  String  errorMsg     = "";

  DEBUG_ESP32_MQTT("B++ 587s: onMessage/ProcRcvd: [" + topic + " " + payload + "]\n");

  // ****** PARSE MESSAGE RECEIVED FROM JMRI
  topicNumParts   = TopicParse (topicLen, &topicCharArray[0], topicDelimiter);
  if (DEBUGMQTT)  {
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
  topicReaderName  = topicParts[4];   readerNum = (readerIDString.indexOf(topicReaderName.charAt(0)) < 0) ? ACK_SENS_NUM : (readerIDString.indexOf(topicReaderName.charAt(0)));
  topicObjectNum   = topicParts[5];
  topicMisc        = topicParts[6];


  // **************** DO WORK BASED ON COMMAND and GROUP *************************
  switch (topicNumParts)  {
    case 0:     // Error
    case 1:     // Error
    case 2:     // Error
      Serial.println("ERROR1-sr239: Improper format message: " + topic + " " + payload);
      break;
    case 3:     // Possible status message from JMRI
      if (topicCommand == "jStat" and topicObjectType == "$state") {
        Serial.println("** JMRI $state last will: " + payload);
        jmri[0].JMRISensor = 0;      //Set the response
        if (JMRITEST == "OFF") JMRIReady = false;           //Only reset the "ready" if not in JMRITEST mode
      }
      else Serial.println("ERROR3-sr247: Unknown command: " + topic + " " + payload);
      break;
    case 4:     // ERROR
      if (topicCommand == "jRbot" and topicObjectType == "REBOOT") {   // *** REBOOT request ***
        ManualReboot();
      }
      else Serial.println("ERROR4-sr253: Improper format message: " + topic + " " + payload);
      break;
    case 5:     // Expected message
      if (topicCommand == "jSend" && topicObjectType == "lt") {
        //Lights specify the reader #
        DEBUG_ESP32_MQTT("B++190s: Rcvd: Light: " + String(readerNum) + " " + payload + "\n");
        LightRcvd(readerNum, payload);
      }
      else Serial.println("ERROR3-sr261: Unknown command: " + topic + " " + payload);
      break;
    case 6:
    case 7:
    default:
      Serial.println("ERROR-sr266: Unknown parsing: " + topic + " " + payload);
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
void LightRcvd (uint8_t lNum, String lState) {       //lNum = JMRI Light number (used for RFID reader re-init)

  if (lNum == ACK_SENS_NUM)  {                       //Light = [large number] is JMRI Activate switch
    if (lState == "ON")         {
      JMRIReady = true;
      Serial.println ("!! JMRI READY Received: " + String(lNum));
      MQTTSendSensor (ACK_SENS_NUM, "ACTIVE");       //Send sensor 0 in response
      mqttHBTime = mainLoopStartTime;                //Reset heartbeat time
    }
    else if (lState == "OFF")         {
      JMRIReady = false;
      Serial.println ("!! JMRI NOT READY Received: " + String(lNum));
      MQTTSendSensor (ACK_SENS_NUM, "INACTIVE");      //Send sensor in response
    }
    else  Serial.println("ERROR-sr318: Incorrect message for Light: " + String(lNum) + " " + lState);
  }
  else if (lNum >= 0)       {                         //Some other JMRI Light command received
    if (lState == "OFF") jmri[lNum].JMRILight = 0;
    else {                   //Re-initialize RFID Reader
      jmri[lNum].JMRILight = 1;
      readers[lNum].InitiateRFIDReader(lNum);
    }
  }
  else Serial.println("ERROR-sr326: Invalid lNum " + String(lNum));
  return;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void MemoryRcvd (uint8_t mNum, String mText) {       //mNUm = Memory number

  if (mNum < 2)  {
    jmri[mNum].JMRIMemory[0] = mText;
  }
  else Serial.println("ERROR-sr336: Invalid mNum received: " + String(mNum));
  return;

}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#endif   //END MQTT Functions
// **************************************************************************************
