//Functions for ESP32_RFID

//Emulates MERG Concentrator for RFID communication with JMRI over WiFi
//REQUIRED MODIFICATION TO MFRC522.cpp when using ESP32 libraries 2.0.8: change F("x") when occurring in return statements to ((__FlashStringHelper *) "x")

/* Code provided "AS IS" with no warranties! PLEASE DO NOT DISTRIBUTE FURTHER WITHOUT PERMISSION OF AUTHOR
   Distributed subject to license as specified in the Github reposoitory
   Code on Github may be modified or withdrawn at any time

   Based on sketch from Thomas Seitz in jmriusers @ groups.io  04-2023  (minor modifications for ESP32)
   Copyright (c) 2023 Jerry Grochow
*/

bool ClientConnected()   {
  //Verify JMRI connected
  bool cc  = false;
  
  if (!client.connected()) {
    WiFiClient newClient = server.available();
    if (newClient) {
      client = newClient;
      Serial.print("New client connected: ");
      Serial.println(newClient);
      cc = true;
    }
  }
  else cc = true;
  
  return(cc); 
}


// ****************************************************************************************
uint8_t InitiateRFIDReaders ()      {
  //Initialize RFID readers
  uint8_t numReaders = 0;

  if (DEBUG)      {
    Serial.print("Default MOSI: ");    Serial.print(MOSI);
    Serial.print("  Default MISO: ");  Serial.print(MISO);
    Serial.print("  Default SCK: ");   Serial.println(SCK);
  }
  for (uint8_t ii = 0; ii < numPossibleReaders; ii++) {
    readers[ii].ssPin = SS_Pins[ii];
    readers[ii].rstPin = RST_Pins[ii];
    readers[ii].id = readerID[ii];
    if (DEBUG) Serial.println("DEBUG: SS: " + String (readers[ii].ssPin) + "  RS: " + String(readers[ii].rstPin));
    //Start Serial Peripheral Interface for this system (chip) select pin
    //#ifdef ESP32
    //    pinMode(readers[ii].ssPin, OUTPUT); digitalWrite(readers[ii].ssPin, LOW);  //enable SPI system select pin
    //#endif
    SPI.begin(SCK, MISO, MOSI, readers[ii].ssPin);
    delay(250);
    readers[ii].mfrc522 = MFRC522(readers[ii].ssPin, readers[ii].rstPin);
    readers[ii].mfrc522.PCD_Init(readers[ii].ssPin, readers[ii].rstPin);
    // Check if the reader is connected
    delay(250);
    if (readers[ii].mfrc522.PCD_PerformSelfTest()) {
      numReaders += 1;
      readers[ii].isConnected = true;
      readers[ii].mfrc522.PCD_SetAntennaGain(readers[ii].mfrc522.RxGain_max);    // Print debugging information
      Serial.print("Reader ");
      Serial.print(readers[ii].id);
      Serial.print(" detected on SS pin ");
      Serial.println(readers[ii].ssPin);
    }
    else {
      readers[ii].isConnected = false;
    }
    //#ifdef ESP32
    //    digitalWrite(readers[ii].ssPin, HIGH);
    //#endif
  }  //END numPossibleReaders
  Serial.print("Number of readers detected: ");
  Serial.print(numReaders);
  Serial.print((numPossibleReaders - numReaders) ? " *NOT EQUAL* " : " EQUAL ");
  Serial.println(numPossibleReaders);

  // Blink the LED to indicate the number of detected readers
  pinMode(LED_BUILTIN, OUTPUT);
  for (int ii = 0; ii < numReaders; ii++) {
    if (readers[ii].isConnected) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(500);
      digitalWrite(LED_BUILTIN, LOW);
      delay(300);
    }
  }

  return (numReaders);

}  //END InitiateRFIDReaders

// **********************************************************************************************

bool ReadRFIDSend (uint8_t ri)      {

  bool cardFound  = false;

  if (readers[ri].isConnected)    {
    //#ifdef ESP32
    //      digitalWrite(readers[ri].ssPin, LOW);
    //#endif
    //If connected, then try to read...
    if (readers[ri].mfrc522.PICC_IsNewCardPresent() && readers[ri].mfrc522.PICC_ReadCardSerial()) {
      cardFound = true;
      Serial.print("Reading tag on reader ");
      Serial.println(readerID[ri]);
      for (uint8_t rj = 0; rj < readers[ri].mfrc522.uid.size; rj++) {
        readers[ri].nuid[rj] = readers[ri].mfrc522.uid.uidByte[rj];
      }
      //#ifdef ESP32
      //        digitalWrite(readers[ri].ssPin, HIGH);
      //#endif

      byte checksum = readers[ri].nuid[0];
      for (uint8_t rj = 1; rj < 5; rj++) {
        checksum ^= readers[ri].nuid[rj];
      }

      // Debugging output for Serial Monitor
      if (DEBUG || !clientPresent)    {              //Make sure to print to console if no JMRI
        Serial.print("DEBUG: ");
        Serial.print(clientPresent);
        Serial.print("  Reader: ");
        Serial.write(readers[ri].id);
        for (uint8_t rj = 0; rj < 5; rj++) {
          Serial.print(readers[ri].nuid[rj] < 0x10 ? "0" : "");
          Serial.print(readers[ri].nuid[rj], HEX);
        }
        Serial.print(" checksum: ");
        Serial.print(checksum < 0x10 ? "0" : "");
        Serial.println(checksum, HEX);
      }

      // Send data to the connected client
      if (clientPresent)      {
        client.write(readers[ri].id);
        for (uint8_t rj = 0; rj < 5; rj++) {
          client.print(readers[ri].nuid[rj] < 0x10 ? "0" : "");
          client.print(readers[ri].nuid[rj], HEX);
        }
        client.print(checksum < 0x10 ? "0" : "");
        client.print(checksum, HEX);

        client.write(0x0D); // CR
        client.write(0x0A); // LF
        client.write('>');
      }

      //Clear readers
      readers[ri].mfrc522.PICC_HaltA();
      readers[ri].mfrc522.PCD_StopCrypto1();

    }  //END new card detected
  }  //END isConnected

  return (cardFound);
}
