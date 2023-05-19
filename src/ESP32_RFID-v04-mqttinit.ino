//Functions for ESP32_RFID

//Emulates MERG Concentrator for RFID communication with JMRI over WiFi
//REQUIRED MODIFICATION TO MFRC522.cpp when using ESP32 libraries 2.0.8: change F("x") when occurring in return statements to ((__FlashStringHelper *) "x")

/* Code provided "AS IS" with no warranties! PLEASE DO NOT DISTRIBUTE FURTHER WITHOUT PERMISSION OF AUTHOR
   Distributed subject to license as specified in the Github reposoitory
   Code on Github may be modified or withdrawn at any time

   Based on sketch from Thomas Seitz in jmriusers @ groups.io  04-2023  (minor modifications for ESP32)
   Copyright (c) 2023 Jerry Grochow
*/

#ifdef JMRIMQTT
//*********** Following functions for initializing MQTT ****************************************************
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool InitializeMQTT()      {

  String fullTopic;

  mqttclient.begin(brokerclient);                //link to wifi object
  mqttclient.onMessage(MqttProcessRcvdMsg);      //Set up to processing received messages
  mqttclient.setKeepAlive(60);                   //Tell broker to have longer than default (seconds)   TEST
  fullTopic = mqttChannel + mqttWillTopic + mqttClientID;
  mqttclient.setWill(fullTopic.c_str(), mqttWillMessage);
  ConnectMQTTBroker();                           //Connect to broker (if not able to connect, will reboot ESP32
  while (!mqttclient.connected()) {}             //WAIT for connect to MQTT broker (shsould always be connected at this point)
  numActiveCommMethods += 1;                     //Increase number of active communication methods
  mqttHBTime = millis();                         //Set up
  if (!SubscribeMQTT()) Serial.println ("***Problem initializing retained MQTT messages");

  return (true);
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool ConnectMQTTBroker()                  {  //Check if MQTT connected

  // Use MDNS to find MQTT Broker IP
  Serial.print("** Start MDNS ");
  for (uint8_t w = 0; w < 10; w++) {
    if (MDNS.begin(clientName)) break;
    Serial.print (".");
  }
  Serial.println("");
  if (!MDNS.begin(clientName)) {
    Serial.println("***   ERROR setting up MDNS responder!");
    //return (false);                    ** Continue on anyway
  }

  Serial.println("** Connecting to MQTT");
  mqttBrokerIP[0] = SelectMQTTBroker(mqttBrokerNum);          //Get perhaps new IP address of broker

  if (DEBUGM) {
    Serial.print("T::CMB519s: Checking if connected: " + String(millis()) + "   ");
    Serial.println(mqttBrokerIP[0]);
  }

  for (uint8_t lp = 0; lp < 9; lp++) {
    if (mqttclient.connect(mqttClientID.c_str())) {
      Serial.print("** MQTT connected to: ");
      Serial.print(mqttBrokerIP[0]);      //Current broker address always stored in IP[0]
      Serial.println(":" + String(mqttPort));
      break;
    }
    else  {                             //Not yet connected to broker
      mqttBrokerNum = (mqttBrokerNum + 1) % numBrokerIP;              //Advance broker try by one
      Serial.print ("    Attempting connection to broker on: ");
      Serial.println(mqttBrokerIP[mqttBrokerNum]);
      mqttBrokerIP[0] = SelectMQTTBroker(mqttBrokerNum);    //Get perhaps new IP address of broker
    }
  } //end for

  if (!mqttclient.connected()) {       //*** REBOOT REBOOT ***//
    //    server.handleClient();       //FUTURE: Check if Web OTA Update before rebooting
    ManualReboot();
  }

  return (true);
}


//+++++++++++++++++++++++++++ Look for MQTT Broker +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
IPAddress SelectMQTTBroker(uint8_t bNum)                   {  //Locate IP and Port

  IPAddress tryMQTTBroker;

  if (DEBUGM) Serial.println("T::SMB554s: " + String(bNum));

  if (bNum != 0)  {
    tryMQTTBroker = mqttBrokerIP[bNum];
  }
  else      {        //If first time, try looking at MDNS.queryService
    uint8_t n = MDNS.queryService("_mqtt", "_tcp");
    Serial.println("    Looking for MQTT Broker broadcast IP");
    if (n == 0) {               //no broker found
      //      delay (oneSec);
      n = MDNS.queryService("_mqtt", "_tcp");      //try one more time
    }
    switch (n)       {        //See how many brokers found, if any
      case 0:
        Serial.println("     No broadcast.");
        break;
      case 1:    //one broker found
        {
          IPAddress firstIP = MDNS.IP(0);       //Check first IP
          if (firstIP[0] == 0)  {               //Just check first part of IP address
            Serial.println ("*** Broadcast Broker IP = null.");
          }
          else  {
            Serial.print("*** Broker IP broadcast: ");
            Serial.print(MDNS.hostname(0));
            Serial.print(" on IP ");
            Serial.print(firstIP);
            Serial.println("");
            tryMQTTBroker = firstIP;                //RETURN broker found
          }
          break;
        }
      default:                             //more than one found
        Serial.println(String (n) + " brokers found.  Using first found: ");
        for (uint8_t i = 0; i < n; ++i) {
          // Print details for each service found
          Serial.print("  ");
          Serial.print(i + 1);
          Serial.print(": ");
          Serial.print(MDNS.hostname(i));
          Serial.print(" (");
          Serial.print(MDNS.IP(i));
          Serial.print(":");
          Serial.print(MDNS.port(i));
          Serial.println(")");
        }
        tryMQTTBroker = MDNS.IP(0);                //RETURN first IP in list
        break;
    }  //end of switch
  } //end of brokerNum = 0

  //Set up to see if broker found at this address
  mqttclient.setHost(tryMQTTBroker, mqttPort);
  return (tryMQTTBroker); //

}   //end of subroutine


//++++++++++++++++ Do MQTT Subscriptions and initializations +++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool SubscribeMQTT ()                 {

  String fullTopic;
  bool mqPub[4];

  Serial.println("** MQTT Resets and Subscriptions");

  //Send MQTT initization messages to clear out prior retained messages
  fullTopic = mqttChannel + mqttWillTopic; fullTopic.remove(fullTopic.length() - 1, 1); //Strip off final slash
  mqPub[0] = mqttclient.publish(fullTopic.c_str(), "", true, 0);   //JMRI state: Retained = true; QoS = 0
  Serial.println("    Reset MQTT object: " + fullTopic);
  fullTopic = mqttChannel + mqttSubscribeTopics[SS_LT] + mqttClientID + JMRI_LT_ACT;
  mqPub[1] = mqttclient.publish(fullTopic.c_str(), "OFF", true, 0);    //Light 0: JMRI set MC ready   ***FOR TESTING WITHOUT JMRI, set to: ON
  Serial.println("    Reset MQTT object: " + fullTopic);
  fullTopic = mqttChannel + mqttPublishTopics[SS_SENS] + mqttClientID + JMRI_SENS_ACK;
  mqPub[2]  = mqttclient.publish(fullTopic.c_str(), "", true, 0);    //Retained = true; QoS = 0
  Serial.println("    Reset MQTT object: " + fullTopic);
  fullTopic = mqttChannel + mqttSubscribeTopics[SS_MEM] + mqttClientID + JMRI_MEM_ESP32_IP;
  mqPub[2]  = mqttclient.publish(fullTopic.c_str(), "", true, 0);    //Memory: Retained = true; QoS = 0
  Serial.println("    Reset MQTT object: " + fullTopic);
  //Put IP address in JMRI memory for easy access
  mqPub[3] = mqttclient.publish(mqttChannel + mqttPublishTopics[SS_MEM] + mqttClientID + JMRI_MEM_ESP32_IP, WiFi.localIP().toString(), true, 0);  //Retaned = true

  //Do required MQTT subscription after RESETs
  fullTopic = mqttChannel + "Jstat/#";     //Status messages from JMRI
  mqttclient.subscribe(fullTopic);
  Serial.println("    Subscribing to " + fullTopic);
  for (uint8_t st = 0; st < 2; st++)    {
    fullTopic = mqttChannel + mqttSubscribeTopics[st] + mqttClientID + "/#";
    Serial.println("    Subscribing to " + fullTopic);
    //bool subscribe(const char topic[], uint8_t qos);
    mqttclient.subscribe(fullTopic);
  }

  return (mqPub[0] && mqPub[1] && mqPub[2] && mqPub[3]);
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//******************************************************************************************************
#endif   //END JMRI MQTT-specific functions
//****************************************************************************************************
// *=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
