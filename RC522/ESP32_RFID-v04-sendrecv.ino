//Functions for ESP32_RFID

//Emulates MERG Concentrator for RFID communication with JMRI over WiFi
//REQUIRED MODIFICATION TO MFRC522.cpp when using ESP32 libraries 2.0.8: change F("x") when occurring in return statements to ((__FlashStringHelper *) "x")

/* Code provided "AS IS" with no warranties! PLEASE DO NOT DISTRIBUTE FURTHER WITHOUT PERMISSION OF AUTHOR
   Distributed subject to license as specified in the Github reposoitory
   Code on Github may be modified or withdrawn at any time

   Based on sketch from Thomas Seitz in jmriusers @ groups.io  04-2023  (minor modifications for ESP32)
   Copyright (c) 2023 Jerry Grochow
*/


//**************************************************************************************
//**************************************************************************************
#ifdef JMRIRFID
//**************************************************************************************
//**************************************************************************************
bool JmriClientConnected()   {
  //Verify JMRI connected to wifiserver
  bool cc  = false;
  if (!jmrirfidclient.connected()) {
    WiFiClient newclient = wifiserver.available();
    if (newclient) {
      jmrirfidclient = newclient;
      Serial.print("** New JMRI client connected: ");
      Serial.println(jmrirfidclient);
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
    jmrirfidclient.write(readers[ri].id);
    for (uint8_t rj = 0; rj < 5; rj++) {
      jmrirfidclient.print(readers[ri].nuidHex[rj] < 0x10 ? "0" : "");
      jmrirfidclient.print(readers[ri].nuidHex[rj], HEX);
    }
    jmrirfidclient.print(csum < 0x10 ? "0" : "");
    jmrirfidclient.print(csum, HEX);

    jmrirfidclient.write(0x0D); // CR
    jmrirfidclient.write(0x0A); // LF
    jmrirfidclient.write('>');

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

  fullTopic = mqttChannel + mqttPublishTopics[SS_REPORT] + mqttClientID + "/" + String(readers[rNum].id);
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
  else                           fullTopic = mqttChannel + mqttPublishTopics[SS_SENS] + mqttClientID + "/" + String(readers[rNum].id);
  mqPub = mqttclient.publish(fullTopic.c_str(), sMsg.c_str());
  delay(veryShortWait);
  if (DEBUGM) Serial.println("B++309s: MQTTSendSensor: " + String(mqPub) + " [" + fullTopic + " " + sMsg + "]");
  return (mqPub);

}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool MQTTSendMemory (uint8_t rNum)    {     //Send one memory

  bool mqPub = false;
  String fullTopic;

  fullTopic = mqttChannel + mqttPublishTopics[SS_MEM] + mqttClientID + "/" + String(readers[rNum].id);
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


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#endif   //END MQTT Functions
//**************************************************************************************
