// ************************* COMMON FUNCTIONS **********************************************

// *********************** Connect to WiFi ****************************************************
bool IsWiFiConnected() {  //Check if WiFi connected

  for (uint8_t lp = 10; lp > 0; lp--) {
    if (wifimulti.run() == WL_CONNECTED) {
      Serial.print("OK: WiFi connected to ");
      Serial.print(WiFi.SSID());
      Serial.print(" on IP address: ");
      Serial.println(WiFi.localIP());
      break;
    } else {
      Serial.print(".");
      Serial.print(lp);
      delay(oneSec);
    }
  }

  if (wifimulti.run() != WL_CONNECTED) {
    ManualReboot("NO WIFI AVAIL");  // *** REBOOT ESP32 ***
  }

  return (true);
}



//++++++++++++++++++++++++++++ Scan Networks (currently only invoked in DEBUG mode +++++++++++++++++++++++++++++++++++++++++++
void ScanNetworks() {
  Serial.println(F("** Scanning for WiFi Networks**"));
  int numSsid = WiFi.scanNetworks();
  if (numSsid == -1) {
    Serial.println("Couldn't get a wifi connection");
    while (true)
      ;
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


void BlinkBuiltIn(uint8_t bl, uint8_t mult) {

  unsigned int duration = 350;

  if (bl == 0) delay(duration * mult);
  else {
    for (uint8_t bi = 0; bi < bl; bi++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(duration * mult);
      digitalWrite(LED_BUILTIN, LOW);
      delay(duration);
    }
  }
}


// **************************************************************************************************
void ManualReboot(String msg) {  //CAN BE REQUESTED VIA MQTT MESSAGE OR AS A RESULT OF NOT CONNECTING TO MQTT

  Serial.println("**E**527c: Manual REBOOT -- " + msg);

#ifdef JMRIMQTT
  if (mqttBrokerPresent) {
    String fullTopic = mqttChannel + mqttPublishTopics[SS_GOODBYE] + mqttClientID;
    unsigned int arrayLength = msg.length() + 1;
    char charMsg[arrayLength];
    msg.toCharArray(charMsg, arrayLength, 0);
    mqttclient.setWill(fullTopic.c_str(), charMsg);  //Reset will to indicate why rebooted
  }
#endif

  OTAserver.handleClient();  //Check if Web OTA Update before rebooting
  BlinkBuiltIn(3, 1);        //SOS
  BlinkBuiltIn(0, 3);        //space
  BlinkBuiltIn(3, 3);
  BlinkBuiltIn(0, 3);  //space
  BlinkBuiltIn(3, 1);
  Serial.println("");
  delay(2 * oneSec);
  ESP.restart();  // *** REBOOT ESP32 ***
}

// ************** END COMMON FUNCTIONS *************************************************
