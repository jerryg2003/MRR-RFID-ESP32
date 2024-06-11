//*** Web OTA Startup ************
//From https://randomnerdtutorials.com/esp32-over-the-air-ota-programming/  2022-12-26


bool WebOTASetup () {
  /*use mdns for host name resolution*/
  Serial.print("** OTA Web Server setup ");
  for (uint8_t w = 0; w < 10; w++) {
    if (MDNS.begin(OTAhost)) break;
    Serial.print (".");
    Serial.print (w);
  }
  if (!MDNS.begin(OTAhost)) {
    Serial.println("***   ERROR setting up WebOTA MDNS responder!");
    return (false);
  }

  Serial.println();
  Serial.println("    mDNS responder started for Web OTA");
  /*return index page which is stored in serverIndex */
  OTAserver.on("/", HTTP_GET, []() {
    OTAserver.sendHeader("Connection", "close");
    OTAserver.send(200, "text/html", loginIndex);
  });
  OTAserver.on("/serverIndex", HTTP_GET, []() {
    OTAserver.sendHeader("Connection", "close");
    OTAserver.send(200, "text/html", serverIndex);
  });
  /*handling uploading firmware file */
  OTAserver.on("/update", HTTP_POST, []() {
    OTAserver.sendHeader("Connection", "close");
    OTAserver.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = OTAserver.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("***UPLOAD: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("***Upload SUCCESS: %u\n\nREBOOTING...\n\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });

  OTAserver.begin();
  return (true);
}
