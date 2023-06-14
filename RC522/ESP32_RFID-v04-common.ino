//Functions for ESP32_RFID

//Emulates MERG Concentrator for RFID communication with JMRI over WiFi
//REQUIRED MODIFICATION TO MFRC522.cpp when using ESP32 libraries 2.0.8: change F("x") when occurring in return statements to ((__FlashStringHelper *) "x")

/* Code provided "AS IS" with no warranties! PLEASE DO NOT DISTRIBUTE FURTHER WITHOUT PERMISSION OF AUTHOR
   Distributed subject to license as specified in the Github reposoitory
   Code on Github may be modified or withdrawn at any time

   Based on sketch from Thomas Seitz in jmriusers @ groups.io  04-2023  (minor modifications for ESP32)
   Copyright (c) 2023 Jerry Grochow
*/

//************************* COMMON FUNCTIONS **********************************************

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
        Serial.println(id);
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
        if (!JmriRFIDSend(ri, checksum))   Serial.println(String(id) + ": ** JMRI RFID connection no longer active: not reported " + tagIDString);
#elif defined(JMRIMQTT)
        if (!JMRIReady)  Serial.println(String(id) + ": ** JMRI not ready: tag not reported " + tagIDString);
        else  {
          jmri[ri].prevJMRIReporter = jmri[ri].JMRIReporter; jmri[ri].JMRIReporter = tagIDString; //Put tag ID in Reporter
          if (!MQTTSendReporter(ri))   Serial.println(String(id) + ": ** MQTT broker lost connection: " + tagIDString + " tag not reported");
          jmri[ri].prevJMRISensor = jmri[ri].JMRISensor; jmri[ri].JMRISensor = 1;                     //ACTIVE
          if (!MQTTSendSensor(ri, "")) Serial.println(String(id) + ": ** JMRI Sensor not set for " + tagIDString);
          jmri[ri].prevJMRIMemory = jmri[ri].JMRIMemory; jmri[ri].JMRIMemory = tagIDString;
          if (!MQTTSendMemory(ri))     Serial.println(String(id) + ": ** JMRI Memory not set for " + tagIDString);
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


//**************************************************************************************************
void ManualReboot()    {               //CAN BE REQUESTED VIA MQTT MESSAGE OR AS A RESULT OF NOT CONNECTING TO MQTT

  Serial.println("**E**185m: Manual REBOOT request");
  delay (5 * oneSec);
  Serial.println("");
  delay(2 * oneSec);
  ESP.restart();                    //*** REBOOT ESP32 ***
}

// ************** END COMMON FUNCTIONS *************************************************
