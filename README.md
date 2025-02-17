# MRR-RFID-ESP32 RFID for Model Railroads
Multiple configurations under development:

Using the MFRC522, PN532, or PN5180 readers:
(1) Emulates MERG Concentrator for RFID communication with JMRI over WiFi (setup: JMRI RFID Connection)
(2) Uses MQTT for RFID communication with JMRI over WiFi (setup: JMRI MQTT Connection)
(3) Uses SPI interface to arduino or ESP32

Using the PN5180 reader:
(1) Slightly modified libraries to get this working more effectively with the ESP32

Hardware:  ESP32-WROOM Development Board (includes WiFi support), either MFRC522, PN532, or PN5180 RFID Readers

Software:  Arduino IDE, various libraries (see the list in the sketches), JMRI. Configurable sketch:
- Select which layout config to use (your own layout config info, test info, etc.)
- Select which type of reader to use (PN5180, RC522, PN532)
- Select which communications method to use (MQTT, JMRI RFID Connection)

  ** V12p: Various updates and minor improvements
  
  ** V11p: Adds Over-The-Air updating and various other improvements (SUPERSEDED by V12p)

  ** V10p: FIRST PUBLIC RELEASE **
  
  Code provided "AS IS" with no warranties! PLEASE DO NOT DISTRIBUTE FURTHER WITHOUT NOTIFICATION TO AUTHOR
   AS CHANGES MAY BE MADE BY AUTHOR AT ANY TIME.
  
  Distributed subject to license as specified in the Github reposoitory. Code on Github may be modified or withdrawn at any time

  Initial version motivated by sketches from Thomas Seitz https://github.com/TMRCI-DEV1/RFID-Concentrators

  Copyright (c) 2023, 2024, 2025 Jerry Grochow

