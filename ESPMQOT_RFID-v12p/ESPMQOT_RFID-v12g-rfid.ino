//Functions for ESP32_RFID

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
  uint8_t nDetRdr = 0;

  Serial.println("** Initialize RFID Readers");
  Serial << "    Default MOSI: " << MOSI << "  Default MISO: " << MISO << "  Default SCK: " << SCK << endl;

  for (uint8_t ii = 0; ii < numPossibleReaders; ii++) {
    if (readers[ii].InitiateRFIDReader(ii))  nDetRdr += 1;
    readerIDString += RDR_ID[ii];  //Create String of first characters of reader names (for later use in processing MQTT messages)
  }  //END numPossibleReaders

  Serial.print ((numPossibleReaders == nDetRdr) ? "OK: " : "!!! ");
  Serial << "Number of readers detected: " << nDetRdr;
  Serial.print((numPossibleReaders == nDetRdr) ? " EQUAL " : " *NOT EQUAL* ");
  Serial << numPossibleReaders << " readers expected" << endl;

  return (nDetRdr);

}  //END InitiateRFIDReaders


// ************************************************************************************************
bool RFIDReader::InitiateRFIDReader(uint8_t ii)   {

  id      = RDR_ID[ii];
  isoType = ISO_Type[ii];
#ifdef UsePN532
  ssPin   = SS_Pins[ii];          //Used by PN532 and RC522
  irqPin  = IRQ_Pins[ii];         //EXPERIMENT
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

  Serial << (F("    Reader: ")) << id  << (F(" Type: ")) << ISOTypesNames[isoType] << (F("  Sensor pin: ")) <<
#ifdef UsePN5180
         nssPin << (F("  Busy pin: ")) << busyPin << (F("  Reset pin: ")) << rstPin << endl;
#endif
#ifdef UseRC522
  ssPin << (F("  Reset pin: ")) << rstPin << endl;
#endif

#ifdef UsePN532
  ssPin << (F("  Interrupt pin: ")) << irqPin << endl;


  if (ssPin == 0) return (false);        //No reader attached to this pin
  SPI_nfc.begin(PN532_SCK, PN532_MISO, PN532_MOSI, ssPin);   //NEW: as recommended by Git forum
  nfc14443_532 = Adafruit_PN532(ssPin, &SPI_nfc);

  nfc14443_532.reset();
  delay(150);
  nfc14443_532.begin();
  uint32_t versiondata = nfc14443_532.getFirmwareVersion();
  if (DEBUGRFOT) Serial << "==VersData:" << nfc14443_532.getFirmwareVersion() << endl;
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
    if (!nfc14443_5180.init14443()) Serial.println("!! ERROR Initializing Reader: " + String(ii) + " " + ISOTypesNames[isoType]); //Includes loading RF and turning on
  }
  else if (isoType == TYPE15693PN5180)        {
    //nfc15693_5180 = PN5180ISO15693(nssPin, busyPin, rstPin, SPI_nfc);        // ** DOESN't WORK **
    nfc15693_5180.begin(nssPin, busyPin, rstPin);           //Modified library to include arguments in begin 2023-06-08
    nfc15693_5180.reset();          //60ms delay in library code
    delay(100);
    nfc15693_5180.setupRF();      //May take up to 500ms: load config and turn on
  }
  else {
    Serial.print(F("!! Unknown Reader Type: ")); Serial.print(ii); Serial.print("  "); Serial.println(isoType);
    readerConnected = false;
  }

  uint8_t readerVersions[] = {0,0};
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
    readerVersions[0] = 0; readerVersions[1] = 0;
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
  mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);  //[No noticable effect of having this here vs after self test]
  mfrc522.PCD_Init(ssPin, rstPin);
  // Check if the reader is connected
  delay(250);
  if (mfrc522.PCD_PerformSelfTest()) {
    //mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);  //[No noticable effect of having this here vs beofre init]
    readerConnected = true;
    Serial << "    Reader " << id << " detected on SS pin " << ssPin << endl;
  }
  else readerConnected = false;
#endif

  if (!readerConnected) {
    Serial << (F("!! Initialization failed for reader ")) << id << endl;
  }
  return (readerConnected);
}


// **********************************************************************************************
bool RFIDReader::ReadRFIDSend (uint8_t ri)      {

  /* Logic flow:
             - Check for card (ReadRfid)
             - If card found and within "card dup" interval -> count, then ignore
             - If card found new, process card (SendCardData)
             - If no card found and within "card dup" interval -> ignore (may be false negative)
             - If no card found and outside "card dup" interval -> reset everything, clear data to JMRI (ClearCardData)
  */

  bool cardProcessed = false;
  bool tagDetected = ReadRFID(ri);
	bool outsideInterval = (loopStartTime > (cardFoundTime + cardDupInterval));

  //If tag detected,
  if (tagDetected)        {                         //If card found, do more processing
    if (outsideInterval) {                          //Reset for new card if outside previous dup interval
      //prevChecksum = 0;
      cardFoundTime = loopStartTime;                        //Reset when card found to now
      cardProcessed = SendCardData(ri);                     //Process new card, sets to true
      cardAlreadyProcessed = true;
			//numTimesTagReRead = 0;
    }
    else {                                           //Inside card dup interval
      if (!cardAlreadyProcessed) Serial << (F("!!ERROR: card found,in dup interval, NOT cardAlreadyProcessed: ")) << loopCnt << "/ " << prevChecksum << endl;
			//NOTE:  This isn't really an error: could be a new tag on the next car.  Needs more code with this "if"  (2025-01-06)
      else {      			                            //If SAME TAG already processed*, inside interval, increment count, ignore (*more code needed)
        numTimesTagReRead += 1;
				cardLastSeenTime = loopStartTime;
				if (DEBUGRFOT) Serial << (F("==198rf: card found,in dup interval, cardAlreadyProcessed: ")) << loopCnt << "/" << numTimesTagReRead << endl;
      } //END card already processed
    }  //END inside interval
  }  //END card detected

  // If a tag is not detected,
  else {                                             //If no card found, set the sensor to INACTIVE and clear the reporter if outside interval
    if (outsideInterval) {  //Reset ALL if outside dup interval
      if (cardAlreadyProcessed) {                    //If first time thru...
        //prevChecksum = 0;
        cardFoundTime = 0;
        cardProcessed = ClearCardData (ri);                   //sets to false
        cardAlreadyProcessed = false;
				numTimesTagReRead = 0;
				cardLastSeenTime = 0;                         //MAYBE? Needs review 2025-01-07
      }
    }
    else {                                            //No card, inside interval (may be false skip)
      if (cardAlreadyProcessed)     {                 //Card was just processed
        //DO NOT RESET ANYTHING
        if (DEBUGRFOT) Serial << (F("==217rf: NO card, cardAlreadyProcessed: ")) << loopCnt << endl;
      }   //END cardAlreadyProcessed
    }  //END inside interval
  } //END not card detected

  return (cardProcessed);

}  //END ReadRFIDSend



// **********************************************************************************************
bool RFIDReader::ReadRFID (uint8_t ri)      {

  bool cardFound  = false;
  unsigned long readStart = millis();        //For TIMING and DEBUG
	unsigned long readStloopSt = readStart - loopStartTime;

  if (DEBUGRFOT and cardAlreadyProcessed) Serial << (F("==309rf:")) << id << (F(" Read start-cardAlreadyProcessed / Loop: ")) << loopCnt << (F("  prevChecksum: ")) << prevChecksum << endl;

  if (readerConnected)    {

#ifdef UsePN532
    if (isoType == TYPE14443PN532)     {  //PN532
//      cardFound = nfc14443_532.readPassiveTargetID(PN532_MIFARE_ISO14443A, &nuidHex[0], &numBytesRead, 35); //SET TIMEOUT (ORIG = no timeout) 
      //TEST
      uint16_t debyg1 = 0; uint16_t debyg2 = 0;
      //TEST cardFound = nfc14443_532.readPassiveTargetID(PN532_MIFARE_ISO14443A, &nuidHex[0], &numBytesRead, 35, &debyg1, &debyg2); //SET TIMEOUT (ORIG = no timeout) Won't work at leass than 30
      cardFound = nfc14443_532.readPassiveTargetID(PN532_MIFARE_ISO14443A, &nuidHex[0], &numBytesRead, 30); //SET TIMEOUT (ORIG = no timeout) Won't work at leass than 30
      if (cardFound & TIMING)  {
//         Serial << "532Found: " << loopCnt << " LS: " << loopStartTime << " readStart-loopStart: " << readStloopSt << " Now-readSt: " << millis() - readStart << endl;
         Serial << "532Found: " << loopCnt << " LS: " << loopStartTime << " readStart-loopStart: " << readStloopSt << " Now-readSt: " << millis() - readStart << " PN532: " << debyg1 << " " << debyg2 <<endl;
         //EXPERIMENT
         bool irqResult = digitalRead(irqPin);
				 Serial << "==321c IRQ " << ri << " Time: " << millis() << ":  Pin: " << irqPin << " Read: " << irqResult << endl;		
      }    
      numBytesProcessed = (numBytesRead > 9 ? 10 : numBytesRead);
    }
#endif

#ifdef UsePN5180
    if (isoType == TYPE14443PN5180)     {
      nfc14443_5180.reset();   //TEST: timing without this goes to >600ms
      //if (!nfc14443_5180.init14443()) Serial.println("!!! ERROR Initializing Reader: " + String(ri) + " " + String(isoType));
      //      int8_t cardReadResult = nfc14443_5180.readTypeA(nuidHex, 0);   //From example BUT gives different result than PN532 and RC522
      int8_t cardReadResult = nfc14443_5180.readCardSerial(nuidHex);   //Gives same result as other card reader types
      if (cardReadResult >= 4) {             //Minium length for Type A uid
        if (TIMING) Serial << "5180-43Found: " << loopCnt << " LS: " << loopStartTime << " readStart-loopStart: " << readStloopSt << " Now-readSt: " << millis() - readStart << endl;
        cardFound = true;
        numBytesRead = cardReadResult;
        //?numBytesRead = nfc14443_5180.uid.size;
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
        if (TIMING) Serial << "5180-93Found: " << loopCnt << " LS: " << loopStartTime << " readStart-loopStart: " << readStloopSt << " Now-readSt: " << millis() - readStart << endl;
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
        if (TIMING) Serial << "522Found: " << loopCnt << " LS: " << loopStartTime << " readStart-loopStart: " << readStloopSt << " Now-tS: " << millis() - readStart << endl;
        cardFound = true;
        //Copy UID bytes into structure
        numBytesRead = mfrc522.uid.size;
        numBytesProcessed = (mfrc522.uid.size > 9 ? 10 : numBytesRead);   //Limit from JMRI processing for MERG Concentrator
        for (uint8_t rj = 0; rj < numBytesProcessed; rj++) {
          nuidHex[rj] = mfrc522.uid.uidByte[rj];
        }
      } //END mfrc522 is card present
    } //END if isoType
#endif

    else Serial.println("!!ERROR 311rf: Unknown reader type: " + id + String(isoType));

    //If tag detected...
    if (cardFound)       {
      // Format RFID data (also compute and save checksum)
      FormatRFIDData();
      if (DEBUGRFOT || numActiveCommMethods == 0) {
        unsigned long nowTime = millis();
        Serial << (F("==388rf: Tag detected rdr ")) << id << (F(" Time: ")) << nowTime << (F(" Loop: ")) << loopCnt << (F("  Bytes read: ")) << numBytesRead << (F("  proc: ")) << numBytesProcessed << (cardAlreadyProcessed ? " [CardPresent]" : "") << endl;
        Serial << (F("  tagIDString: ")) << tagIDString << (F(" checksum: ")) << (checksum < 0x10 ? "0" : ""); Serial.print(checksum, HEX);
        Serial << (F("  prevChecksum: ")) << (prevChecksum < 0x10 ? "0" : ""); Serial.print(prevChecksum, HEX);
      }
			if (TIMING)  {
        unsigned long nowTime = millis();
			  Serial << (F("==325rf: End of ReadRFID: Now-readStart: ")) << nowTime - readStart << (F(" Now-loopSt: ")) << nowTime - loopStartTime << endl;
      }
    }   //END cardFound

  } //END readerConnected

  return (cardFound);
}


// **************************************************************************************
bool RFIDReader::SendCardData(uint8_t ri)          {

  /*  Either JmriRfidSend or individual MQTT objects published */
  unsigned long sendStart = millis();                    //For timing test

  digitalWrite(LED_BUILTIN, HIGH);             //Indicator...


  // Send data to the connected client
#ifdef JMRIRFID
  if (!JmriRFIDSend(ri))   Serial.println(String(id) + ": ** JMRI RFID connection not active: not reported " + tagIDString);
#endif
#ifdef JMRIMQTT
  if (!JMRIReady)  Serial.println(String(id) + ": ** JMRI not ready: tag not reported " + tagIDString);
  else  {
    // *** INVESTIGATE: IMPORTANT JMRI ISSUE: Set Sensor ACTIVE before setting Reporter ***
    //Serial << "S: " << millis() << " " << endl;
    jmri[ri].prevJMRISensor = jmri[ri].JMRISensor; jmri[ri].JMRISensor = 1;                     //ACTIVE
    if (!MQTTSendSensor(ri, "")) Serial.println(String(id) + ": ** JMRI Sensor not set for " + tagIDString);
    //delay (10);   // FOR TESTING TIMINIG ISSUE WITH JMRI
    //Serial << "R: " << millis() << " " << endl;
    jmri[ri].prevJMRIReporter = jmri[ri].JMRIReporter; jmri[ri].JMRIReporter = tagIDString; //Put tag ID in Reporter
    if (!MQTTSendReporter(ri))   Serial.println(String(id) + ": ** MQTT broker lost connection: " + tagIDString + " tag not reported");
    //delay (100);   // FOR TESTING TIMINIG ISSUE WITH JMRI
    //Serial << "M: " << millis() << " " << endl;
    jmri[ri].prevJMRIMemory[0] = jmri[ri].JMRIMemory[0]; jmri[ri].JMRIMemory[0] = tagIDString;
    if (!MQTTSendMemory(ri,0))     Serial.println(String(id) + ": ** JMRI Memory not set for " + tagIDString);
 		jmri[ri].prevJMRIMemory[1] = jmri[ri].JMRIMemory[1]; jmri[ri].JMRIMemory[1] = sendStart-loopStartTime;       //Save time to process
    //JMRIMemory[1] will be sent when report cleared since not necessary now
    //Serial << "MQTT Done: " << millis() << " " << endl;
  }  //END else
#endif

  // Output for Serial Monitor
  unsigned long nowTime = millis();
  Serial << id << (F(": Tag processed: ")) << tagIDString << (F(" checksum: ")) << (checksum < 0x10 ? "0" : "");
  Serial.print(checksum, HEX);
  Serial << (F(" Total loop time: ")) << nowTime - loopStartTime << " Reporting time: " << nowTime - sendStart << endl;

  return (true);
}

// **************************************************************************************
bool RFIDReader::ClearCardData(uint8_t ri)          {

  digitalWrite(LED_BUILTIN, LOW);             //Indicator
  unsigned long clearStart = millis();        //For TIMING and DEBUG

#ifdef JMRIMQTT
  if (!JMRIReady)  Serial.println(String(id) + ": ** JMRI not ready: no tag not reported " + tagIDString);
  else {
    // *** IMPORTANT JMRI ISSUE: Set sensor INACTIVE before sending blank reporter ******
    jmri[ri].JMRISensor = 0;
    MQTTSendSensor(ri, "");
    //JMRI will clear "Block Value" when the block sensor shows unoccupied, but sketch has to clear the Reporter
    jmri[ri].JMRIReporter = "";
    MQTTSendReporter(ri);
    //Leave JMRI Memory intact (comment out next two lines)
    //jmri[ri].JMRIMemory[0] = "";
    //MQTTSendMemory(ri, 0);
    //Send time to do read current in JMRIMemory[1]
    MQTTSendMemory(ri,1);
		jmri[ri].prevJMRIMemory[2] = jmri[ri].JMRIMemory[2]; jmri[ri].JMRIMemory[2] = numTimesTagReRead;              //Save # reREads
    MQTTSendMemory(ri,2);
  }  //END else

#endif

#ifdef JMRIRFID
  //Handled automatically by JMRI RFID Connection
#endif

  Serial << id << (F(":   Tag cleared: ")) << tagIDString << (F(" checksum: ")) << (checksum < 0x10 ? "0" : "");
  Serial.print (checksum, HEX);
  unsigned long nowTime = millis();
  Serial << (F(" Rereads: ")) << numTimesTagReRead << (F(" Last seen time: ")) << cardLastSeenTime << (F(" Total loop time: ")) << nowTime - loopStartTime << (F(" Reporting time: ")) << nowTime - clearStart << endl;

  return (false);
}

// **************************************************************************************
void RFIDReader::FormatRFIDData() {                //Creates tagIDString
  String rfidData = "";

  numBytesProcessed = 5;            //** Limitation in the JMRI software 2023-12-26
	//Number of bytes read may be less than this so filled with 0 2024-11-23

  // Add NUID to the RFID data string
  if (isoType == TYPE14443PN532 || isoType == TYPE14443PN5180 || isoType == TYPE14443RC522) {
    for (uint8_t j = 0; j < numBytesProcessed; j++) {
      //Convert Hex a-z to A-Z for compatibility with MFRC522 library
      char hexChars[3] = {'0','0'};  // Array to hold two hex digits and null terminator
			//Don't do conversion if numBtyesRead < 5 2024-11-23
      if (numBytesRead > j) snprintf(hexChars, sizeof(hexChars), "%02X", nuidHex[j]);  //NOTE: function found with help of Bard 2023-12-27
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

  else if (isoType == TYPE15693PN5180)    {     //Read opposide direction
    for (uint8_t j = 0; j < numBytesProcessed; j++) {
      if (nuidHex[7 - j] < 0x10) {         //LSB is first for ISO15693
        rfidData += "0";
      }
      rfidData += String(nuidHex[7 - j], HEX);
    }

    //No checksum calculation ??

  }  //END TYPE15693PN5180


  else  {
    Serial << (F("!!ERROR 456rf: Unknown ISOType: ")) << isoType << endl;
  }

  // Return the formatted RFID data string
  tagIDString = rfidData;
}

// *************************************************************************************************
// ************ END RFID FUNCTIONS *****************************************************************