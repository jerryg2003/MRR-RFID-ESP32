//Functions for ESP32_RFID

//REQUIRED MODIFICATION TO MFRC522.cpp when using ESP32 libraries 2.0.8: change F("x") when occurring in return statements to ((__FlashStringHelper *) "x")

/* Code provided "AS IS" with no warranties! PLEASE DO NOT DISTRIBUTE FURTHER WITHOUT PERMISSION OF AUTHOR
   Distributed subject to license as specified in the Github reposoitory
   Code on Github may be modified or withdrawn at any time

   Based on sketch from Thomas Seitz in jmriusers @ groups.io  04-2023  (minor modifications for ESP32)
   Copyright (c) 2023 Jerry Grochow
*/

// ************************* COMMON FUNCTIONS **********************************************

// *********************** Connect to WiFi ****************************************************
bool IsWiFiConnected()           {  //Check if WiFi connected

  for (uint8_t lp = 10; lp > 0; lp--) {
    if (wifimulti.run() == WL_CONNECTED) {
      Serial.print("** WiFi connected to ");
      Serial.print(WiFi.SSID());
      Serial.print(" on IP address: ");
      Serial.println(WiFi.localIP());
      break;
    }
    else {
      Serial.print(".");
      Serial.print(lp);
      delay(oneSec);
    }
  }

  if (wifimulti.run() != WL_CONNECTED) {
    ManualReboot();             // *** REBOOT ESP32 ***
  }

  return (true);
}


//++++++++++++++++++++++++++++ Scan Networks (currently only invoked in DEBUG mode +++++++++++++++++++++++++++++++++++++++++++
void ScanNetworks ()                {
  Serial.println(F("** Scanning for WiFi Networks**"));
  int numSsid = WiFi.scanNetworks();
  if (numSsid == -1) {
    Serial.println("Couldn't get a wifi connection");
    while (true);
  }
  // print the list of networks seen:
  Serial.print("** Number of available networks:");
  Serial.println(numSsid);

  // print the network number and name for each network found:
  for (int thisNet = 0; thisNet < numSsid; thisNet++) {
    Serial.print("    ");
    Serial.print(WiFi.SSID(thisNet));
    Serial.print(": Signal: ");
    Serial.print(WiFi.RSSI(thisNet));
    Serial.print(" dBm;");
    Serial.print("  Encryption: ");
    int thisType = WiFi.encryptionType(thisNet);
    switch (thisType) {
      case 2:
        Serial.println("WPA");
        break;
      case 3:
        Serial.println("WPA2-Pers");
        break;
      case 4:
        Serial.println("WPA2");
        break;
      case 5:
        Serial.println("WEP");
        break;
      case 7:
        Serial.println("None");
        break;
      case 8:
        Serial.println("Auto");
        break;
      default:
        Serial.println(thisType);
        break;
    }
  }
}  //END of DEBUGS for Scan Networks


// ****************************************************************************************
uint8_t InitiateRFIDReaders ()      {
  //Initialize RFID readers
  uint8_t nDetRdr = 0;

  Serial.println("** Initialize RFID Readers");
  Serial << "    Default MOSI: " << MOSI << "  Default MISO: " << MISO << "  Default SCK: " << SCK << endl;

  for (uint8_t ii = 0; ii < numPossibleReaders; ii++) {
    if (readers[ii].InitiateRFIDReader(ii))  nDetRdr += 1;
    readerIDString += RDR_ID[ii];  //Create String of first characters of reader names (for later use in processing MQTT messages)
  }  //END numPossibleReaders

  Serial << "** Number of readers detected: " << nDetRdr;
  Serial.print((numPossibleReaders == nDetRdr) ? " EQUAL " : " *NOT EQUAL* ");
  Serial << numPossibleReaders << " readers expected" << endl;

  // Blink the LED to indicate the number of detected readers
  BlinkBuiltIn(nDetRdr, 2);

  return (nDetRdr);

}  //END InitiateRFIDReaders


// ************************************************************************************************
bool RFIDReader::InitiateRFIDReader(uint8_t ii)   {

  id      = RDR_ID[ii];
  isoType = ISO_Type[ii];
#ifdef UsePN532
  ssPin   = SS_Pins[ii];          //Used by PN532 and RC522
#endif
#ifdef UseRC522
  ssPin   = SS_Pins[ii];          //Used by PN532 and RC522
  rstPin  = RST_Pins[ii];        //Used by PN5180 and RC522
#endif
#ifdef UsePN5180
  nssPin  = NSS_Pins[ii];        //Used by PN5180
  rstPin  = RST_Pins[ii];        //Used by PN5180 and RC522
  busyPin = BUSY_Pins[ii];      //Used by PN5180
#endif

  Serial << (F("    Reader: ")) << id  << (F(" ISO Type: ")) << ISOTypesNames[isoType] << (F("  Sensor pin: ")) <<
#ifdef UsePN5180
         nssPin << (F(  "Busy pin: ")) << busyPin << (F("  Reset pin: ")) << rstPin << endl;
#endif
#ifdef UseRC522
  ssPin << (F("  Reset pin: ")) << rstPin << endl;
#endif

#ifdef UsePN532
  ssPin << endl;

  if (ssPin == 0) return (false);        //No reader attached to this pin
  SPI_nfc.begin(PN532_SCK, PN532_MISO, PN532_MOSI, ssPin);   //NEW: as recommended by Git forum
  //???  nfc14443_532 = Adafruit_PN532(ssPin, &SPI);
  nfc14443_532 = Adafruit_PN532(ssPin, &SPI_nfc);
  Serial << "**Initializing reader " << id << " on SS pin: " << ssPin << endl;

  nfc14443_532.begin();
  uint32_t versiondata = nfc14443_532.getFirmwareVersion();
  if (!versiondata) {
    // Mark as not detected
    readerConnected = false;
  }
  else {
    readerConnected = true;
    //nfc14443_532.SAMConfig();           //Secure Access Module (optional)
    nfc14443_532.setPassiveActivationRetries(0x88);  //FF means no limit (documentation says only affects I2C connection type)
    // Print information about the detected reader
    Serial.print("    Found chip PN5"); Serial.print((versiondata >> 24) & 0xFF, HEX);
    Serial.print("  Firmware ver. "); Serial.print((versiondata >> 16) & 0xFF, DEC);
    Serial.print('.'); Serial.println((versiondata >> 8) & 0xFF, DEC);
  }
#endif

#ifdef UsePN5180
  //SPI.begin done by PN5180 Library: Start Serial Peripheral Interface for this system (chip) select pin

  if (nssPin == 0) return (false);        //No reader attached to this pin
  if (isoType == TYPE14443PN5180)   {
    //nfc14443_5180 = PN5180ISO14443(nssPin, busyPin, rstPin, SPI_nfc);        // ** DOESN't WORK **
    nfc14443_5180.begin(nssPin, busyPin, rstPin);           //Modified library to include arguments in begin 2023-06-08
    nfc14443_5180.reset();
    delay(100);
    if (!nfc14443_5180.init14443()) Serial.println("*** ERROR Initializing Reader: " + String(ii) + " " + ISOTypesNames[isoType]);
  }
  else if (isoType == TYPE15693PN5180)        {
    //nfc15693_5180 = PN5180ISO15693(nssPin, busyPin, rstPin, SPI_nfc);        // ** DOESN't WORK **
    nfc15693_5180.begin(nssPin, busyPin, rstPin);           //Modified library to include arguments in begin 2023-06-08
    nfc15693_5180.reset();          //60ms delay in library code
    delay(100);
    nfc15693_5180.setupRF();      //May take up to 500ms
  }
  else {
    Serial.print(F("***Unknown Reader Type: ")); Serial.print(ii); Serial.print("  "); Serial.println(isoType);
    readerConnected = false;
  }

  uint8_t readerVersions[2];
  if      (isoType == TYPE14443PN5180)  nfc14443_5180.readEEprom(PN5180_PRODUCT_VERSION, readerVersions, sizeof(readerVersions));
  else if (isoType == TYPE15693PN5180)  nfc15693_5180.readEEprom(PN5180_PRODUCT_VERSION, readerVersions, sizeof(readerVersions));
  else {
    Serial.print(F("***Unknown Reader Type: ")); Serial.print(ii); Serial.print("  "); Serial.println(isoType);
  }
  Serial.print(F("    Product="));
  Serial.print(readerVersions[1]);
  Serial.print(".");
  Serial.print(readerVersions[0]);
  if (0xff == readerVersions[1]) readerConnected = false; // if product version 255, the initialization failed
  else {                        //Print out product info
    readerConnected = true;
    if      (isoType == TYPE14443PN5180)  nfc14443_5180.readEEprom(PN5180_FIRMWARE_VERSION, readerVersions, sizeof(readerVersions));
    else if (isoType == TYPE15693PN5180)  nfc15693_5180.readEEprom(PN5180_FIRMWARE_VERSION, readerVersions, sizeof(readerVersions));
    Serial.print(F("   Firmware="));
    Serial.print(readerVersions[1]);
    Serial.print(".");
    Serial.print(readerVersions[0]);
    if      (isoType == TYPE14443PN5180)  nfc14443_5180.readEEprom(PN5180_EEPROM_VERSION, readerVersions, sizeof(readerVersions));
    else if (isoType == TYPE15693PN5180)  nfc15693_5180.readEEprom(PN5180_EEPROM_VERSION, readerVersions, sizeof(readerVersions));
    Serial.print(F("   EEPROM="));
    Serial.print(readerVersions[1]);
    Serial.print(".");
    Serial.println(readerVersions[0]);
  }
#endif

#ifdef UseRC522
  if (ssPin == 0) return (false);        //No reader attached to this pin
  //  SPI_nfc.begin(SCK, MISO, MOSI, ssPin);    // ** READER WILL NOT INITIATE WITH THIS **  Why?
  SPI.begin(SCK, MISO, MOSI, ssPin);
  delay(200);
  mfrc522 = MFRC522(ssPin, rstPin);
  mfrc522.PCD_Init(ssPin, rstPin);
  // Check if the reader is connected
  delay(250);
  if (mfrc522.PCD_PerformSelfTest()) {
    readerConnected = true;
    mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);
    Serial << "    Reader " << id << " detected on SS pin " << ssPin << endl;
  }
  else readerConnected = false;
#endif

  if (!readerConnected) {
    Serial << (F("** Initialization failed for reader ")) << ii << " " << id << endl;
  }
  return (readerConnected);
}


// **********************************************************************************************
bool RFIDReader::ReadRFIDSend (uint8_t ri)      {

  /* Logic flow:
             - Check for card (ReadRfid)
             - If card found and within "card dup" interval -> ignore
             - If card found new, process card (SendCardData)
             - If no card found and within "card dup" interval -> ignore (may be false negative)
             - If no card found and outside "card dup" interval -> reset everything, clear data to JMRI (ClearCardData)
  */

  bool cardProcessed = false;

  //If tag detected,
  if (ReadRFID(ri))        {                                 //If card found, do more processing
    if (loopStartTime > (cardFoundTime + cardDupInterval)) { //Reset for new card if outside dup interval
      prevChecksum = 0;
      cardFoundTime = loopStartTime;
      cardProcessed = SendCardData(ri);                     //Process new card
      cardAlreadyProcessed = true;
    }
    else {                                                  //Inside card dup interval
      if (!cardAlreadyProcessed) {
        Serial << (F("ERROR: card found,in dup interval, NOT cardAlreadyProcessed: ")) << loopCnt << "/ " << prevChecksum << endl;
      }  //END DEBUG
      else {                     //If already processed, inside interval, ignore
        if (DEBUG) {
          Serial << (F("DEBUG: card found,in dup interval, cardAlreadyProcessed: ")) << loopCnt << "/ " << prevChecksum << endl;
        }  //END DEBUG
      } //END card already processed
    }  //END inside interval
  }  //END card detected
  // If a tag is not detected,
  else {                                                      //If no card found, set the sensor to INACTIVE and clear the reporter if outside interval
    if (loopStartTime > (cardFoundTime + cardDupInterval)) {  //Reset ALL if outside dup interval
      if (cardAlreadyProcessed) {                //If first time thru...
        prevChecksum = 0;
        cardFoundTime = 0;
        cardProcessed = ClearCardData (ri);      //sets to false
        cardAlreadyProcessed = false;
      }
    }
    else {                                                    //No card, inside interval (may be false skip)
      if (cardAlreadyProcessed)     {                           //Card was just processed
        if (DEBUG) {
          Serial << (F("DEBUG: NO card, cardAlreadyProcessed: ")) << loopCnt << endl;
        }
        //DO NOT RESET ANYTHING
      }   //END cardAlreadyProcessed
    }  //END inside interval
  } //END not card detected

  return (cardProcessed);

}  //END ReadRFIDSend


// **********************************************************************************************
bool RFIDReader::ReadRFID (uint8_t ri)      {

  bool cardFound  = false;

  if (DEBUG and cardAlreadyProcessed)  {
    Serial << (F("DEBUG:")) << id << (F("/Read start-cardAlreadyProcessed / Loop: ")) << loopCnt << (F("  prevChecksum: ")) << prevChecksum << endl;
  }

  if (readerConnected)    {

#ifdef UsePN532
    if (isoType == TYPE14443PN532)     {  //PN532
      cardFound = nfc14443_532.readPassiveTargetID(PN532_MIFARE_ISO14443A, &nuidHex[0], &numBytesRead, 30); //SET TIMEOUT (ORIG = no timeout) Won't work at leass than 30
      numBytesProcessed = (numBytesRead > 9 ? 10 : numBytesRead);
    }
#endif

#ifdef UsePN5180
    if (isoType == TYPE14443PN5180)     {
      //      nfc14443_5180.reset();
      //      if (!nfc14443_5180.init14443()) Serial.println("*** ERROR Initializing Reader: " + String(ri) + " " + String(isoType));
      //      int8_t cardReadResult = nfc14443_5180.readTypeA(nuidHex, 0);   //From example BUT gives different result than PN532 and RC522
      int8_t cardReadResult = nfc14443_5180.readCardSerial(nuidHex);   //Gives same result as other card reader types
      if (cardReadResult >= 4) {             //Minium length for Type A uid
        cardFound = true;
        numBytesRead = cardReadResult;
        //numBytesRead = nfc14443_5180.uid.size;
        numBytesProcessed = (numBytesRead > 9 ? 10 : numBytesRead);
      }  //END cardReadResult
    }  //END iso14443

    else if (isoType == TYPE15693PN5180)  {
      // check for ICODE-SLIX2 password protected tag
      //    uint8_t password[] = {0x01, 0x02, 0x03, 0x04}; // put your privacy password here
      //    ISO15693ErrorCode myrc = nfc15693_5180.disablePrivacyMode(password);
      //    if (ISO15693_EC_OK == myrc) Serial.println("disablePrivacyMode successful");
      ISO15693ErrorCode rc = nfc15693_5180.getInventory(nuidHex);
      if (rc == ISO15693_EC_OK) {
        cardFound = true;
        numBytesRead = 8;         //? nfc15693_5180.uid.size;
        numBytesProcessed = 8;    //?? (nfc15693_5180.uid.size > 9 ? 10 : numBytesRead);

        //enable privacy mode
        //ISO15693ErrorCode myrc = nfc15693_5180.enablePrivacyMode(password);
        //if (ISO15693_EC_OK == myrc) Serial.println("enablePrivacyMode successful");


      }  //END tag found
    }  //END iso15693
#endif

#ifdef UseRC522
    if (isoType == TYPE14443RC522)   {
      if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
        cardFound = true;
        //Copy UID bytes into structure
        numBytesRead = mfrc522.uid.size;
        numBytesProcessed = (mfrc522.uid.size > 5 ? 5 : numBytesRead);   //Limit from JMRI processing for MERG Concentrator
        for (uint8_t rj = 0; rj < numBytesProcessed; rj++) {
          nuidHex[rj] = mfrc522.uid.uidByte[rj];
        }
      } //END mfrc522 is card present
    } //END if isoType
#endif

    else Serial.println("** ERROR: Unknown reader type: " + id + String(isoType));

    //If tag detected...
    if (cardFound)       {
      // Format RFID data (also compute and save checksum)
      FormatRFIDData();
      if (DEBUG || numActiveCommMethods == 0 || !JMRIReady) {
        Serial << (F("DEBUG: Tag detected rdr ")) << id << (F(" Loop: ")) << loopCnt << (F("  Bytes read: ")) << numBytesRead << (F("  proc: ")) << numBytesProcessed << (cardAlreadyProcessed ? " [CardPresent]" : "") << endl;
        Serial << (F("  tagIDString: ")) << tagIDString << (F(" checksum: ")) << (checksum < 0x10 ? "0" : ""); Serial.print(checksum, HEX);
        Serial << (F("  prevChecksum: ")) << (prevChecksum < 0x10 ? "0" : ""); Serial.print(prevChecksum, HEX);
        Serial << (F(" Proc time: ")) << millis() - loopStartTime << endl;
      }

    }   //END cardFound

  } //END readerConnected

  return (cardFound);
}


// **************************************************************************************
bool RFIDReader::SendCardData(uint8_t ri)          {

  /*  Either JmriRfidSend or individual MQTT objects published */

  cardFoundTime = loopStartTime;              //Keep track of when card first found

  digitalWrite(LED_BUILTIN, HIGH);             //Indicator...

  // Output for Serial Monitor
  Serial << id << (F(": Tag processed: ")) << tagIDString << (F(" checksum: ")) << (checksum < 0x10 ? "0" : "");
  Serial.print(checksum, HEX);
  Serial << (F(" Proc time: ")) << millis() - loopStartTime << endl;

  // Send data to the connected client
#ifdef JMRIRFID
  if (!JmriRFIDSend(ri))   Serial.println(String(id) + ": ** JMRI RFID connection not active: not reported " + tagIDString);
#endif
#ifdef JMRIMQTT
  if (!JMRIReady)  Serial.println(String(id) + ": ** JMRI not ready: tag not reported " + tagIDString);
  else  {
    jmri[ri].prevJMRISensor = jmri[ri].JMRISensor; jmri[ri].JMRISensor = 1;                     //ACTIVE
    if (!MQTTSendSensor(ri, "")) Serial.println(String(id) + ": ** JMRI Sensor not set for " + tagIDString);
    //delay (100);   // FOR TESTING TIMINIG ISSUE WITH JMRI
    jmri[ri].prevJMRIReporter = jmri[ri].JMRIReporter; jmri[ri].JMRIReporter = tagIDString; //Put tag ID in Reporter
    if (!MQTTSendReporter(ri))   Serial.println(String(id) + ": ** MQTT broker lost connection: " + tagIDString + " tag not reported");
    jmri[ri].prevJMRIMemory[0] = jmri[ri].JMRIMemory[0]; jmri[ri].JMRIMemory[0] = tagIDString;
    if (!MQTTSendMemory(ri, 0))     Serial.println(String(id) + ": ** JMRI Memory not set for " + tagIDString);
  }  //END else
#endif

  return (true);
}

// **************************************************************************************
bool RFIDReader::ClearCardData(uint8_t ri)          {

  digitalWrite(LED_BUILTIN, LOW);             //Indicator

  // Debugging output for Serial Monitor
  if (DEBUG)    {   //Make sure to print to console if no communication to JMRI
    Serial << (F("DEBUG: Tag cleared: tagIDString: ")) << tagIDString << (F(" checksum: ")) << (checksum < 0x10 ? "0" : "");
    Serial.print (checksum, HEX);
    Serial << (F(" Proc time: ")) << millis() - loopStartTime << endl;
  }

#ifdef JMRIRFID
  return (false);             //Handled automatically by JMRI RFID Connection
#endif
#ifdef JMRIMQTT
  if (!JMRIReady)  Serial.println("** JMRI not ready: no tag not reported " + tagIDString);
  else {
    // *** IMPORTANT JMRI ISSUE: Set sensor INACTIVE before sending blank reporter ******
    jmri[ri].JMRISensor = 0;
    if (!MQTTSendSensor(ri, "")) Serial.println("** JMRI Sensor not set for " + tagIDString);
    //JMRI will clear "Block Value" when the block sensor shows unoccupied, but sketch has to clear the Reporter
    jmri[ri].JMRIReporter = "";
    if (!MQTTSendReporter(ri))   Serial.println("** MQTT broker lost connection: " + tagIDString + " not tag not reported ");
    //Leave JMRI Memory intact (comment out next two lines)
    //jmri[ri].JMRIMemory[0] = "";
    //if (!MQTTSendMemory(ri, 0))      Serial.println("** JMRI Memory not set for " + tagIDString);
  }  //END else

  Serial << id << (F(":   Tag cleared: ")) << tagIDString << (F(" checksum: ")) << (checksum < 0x10 ? "0" : "");
  Serial.print (checksum, HEX);
  Serial << (F(" Proc time: ")) << millis() - loopStartTime << endl;

#endif

  return (false);
}

// **************************************************************************************
void RFIDReader::FormatRFIDData() {                //Creates tagIDString
  String rfidData = "";

  //  numBytesProcessed = 5;            //Limitation in the JMRI software 2023-12-26

  // Add NUID to the RFID data string
  if (isoType == TYPE14443PN532 || isoType == TYPE14443PN5180 || isoType == TYPE14443RC522) {
    for (uint8_t j = 0; j < numBytesProcessed; j++) {
      //Convert Hex a-z to A-Z for compatibility with MFRC522 library
      char hexChars[3] = {0};  // Array to hold two hex digits and null terminator
      snprintf(hexChars, sizeof(hexChars), "%02X", nuidHex[j]); //NOTE: found with help of Bard 2023-12-27
      rfidData += hexChars;
    }

    // Calculate the checksum from the reader's NUID
    prevChecksum = checksum;       //Save for comparison
    checksum = nuidHex[0];
    for (uint8_t j = 1; j < numBytesProcessed; j++) {
      checksum ^= nuidHex[j];
    }

    // DO NOT Append checksum to the data string (not needed for MQTT; send independently by JmriRfidSend)
    //if (checksum < 0x10) rfidData += "0";
    //rfidData += String(checksum, HEX);
  }  //END TYPE14443

  else if (isoType == TYPE15693PN5180)    {
    for (uint8_t j = 0; j < numBytesProcessed; j++) {
      if (nuidHex[7 - j] < 0x10) {         //LSB is first for ISO15693
        rfidData += "0";
      }
      rfidData += String(nuidHex[7 - j], HEX);
    }

    //No checksum calculation ??

  }  //END TYPE15693PN5180


  else  {
    Serial.print("ERROR: Unknown ISOType: ");
    Serial.println(isoType);
  }

  // Return the formatted RFID data string
  tagIDString = rfidData;
}

// *************************************************************************************************
void BlinkBuiltIn(uint8_t bl, uint8_t mult)     {

  for (uint8_t bi = 0; bi < bl; bi++)    {
    digitalWrite(LED_BUILTIN, HIGH);
    delay (750 * mult);
    digitalWrite(LED_BUILTIN, LOW);
    delay (350);
  }
}


// **************************************************************************************************
void ManualReboot()    {               //CAN BE REQUESTED VIA MQTT MESSAGE OR AS A RESULT OF NOT CONNECTING TO MQTT

  Serial.println("**E**185m: Manual REBOOT request");
  BlinkBuiltIn(5, 1);
  Serial.println("");
  delay(2 * oneSec);
  ESP.restart();                    // *** REBOOT ESP32 ***
}

// ************** END COMMON FUNCTIONS *************************************************
