# MRR-RFID-ESP32 RFID for Model Railroads
Multiple configurations under development:

For the MFRC522 reader:
(1)Emulates MERG Concentrator for RFID communication with JMRI over WiFi (setup: JMRI RFID Connection)
(2)Uses MQTT for RFID communication with JMRI over WiFi (setup: JMRI MQTT Connection)

For the PN5180 reader:
(1) Test sketch to get the both ISO protocols working (15693 and 14443)
(2) Slightly modified libraries to get this working more effectively with the ESP32
(3) TBD: sketch to communicate via MQTT to JMRI. (expected by June 30, 2023)

Hardware:  ESP32-WROOM Development Board (includes WiFi support), either MFRC522 or PN5180 RFID Readers

Software:  Arduino IDE, various libraries (see the list in the sketches), JMRI 

  ** FIRST PUBLIC RELEASE **
  
  Code provided "AS IS" with no warranties! PLEASE DO NOT DISTRIBUTE FURTHER WITHOUT PERMISSION OF AUTHOR
   AS CHANGES MAY BE MADE BY AUTHOR AT ANY TIME.
  
  Distributed subject to license as specified in the Github reposoitory. Code on Github may be modified or withdrawn at any time

  Motivated by sketches from Thomas Seitz https://github.com/TMRCI-DEV1/RFID-Concentrators

  Copyright (c) 2023 Jerry Grochow

