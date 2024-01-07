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
// *********** Following functions for initializing MQTT ****************************************************
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

  // Setup MDNS to find MQTT Broker IP
  Serial.print("** Start MDNS Search ");
  for (uint8_t w = 0; w < 10; w++) {
    if (!MDNS.begin(clientName)) Serial.print (".");
    else break;
  }
  Serial.println("");
  if (!MDNS.begin(clientName))  Serial.println("***   ERROR setting up MDNS responder!");

  //Try all this twice and if not successful, reboot
  for (uint8_t tb = 0; tb < 2; tb++)          {

    mqttBrokerIP[0] = SearchMQTTBroker();    //Get perhaps new IP address of broker

    for (uint8_t lp = 0; lp < numBrokerIP; lp++) {
      if (mqttBrokerIP[lp] != 0)  {
        Serial.print ("    Attempting connection to broker on: ");
        Serial.println(mqttBrokerIP[lp]);
        mqttclient.setHost(mqttBrokerIP[lp], mqttPort);
        if (mqttclient.connect(mqttClientID.c_str())) {
          Serial.print("** MQTT connected to: ");
          Serial.print(mqttBrokerIP[lp]);      //Current broker address always stored in IP[0]
          Serial.println(":" + String(mqttPort));
          break;
        }
      } //END non-blank IP
    } //end cycle thru broker IPs

    if (mqttclient.connected()) break;   //If connected, no need to try again.
  }  //END Try Twice

  if (!mqttclient.connected()) {       // *** REBOOT REBOOT ***//
    //    server.handleClient();       //FUTURE: Check if Web OTA Update before rebooting
    ManualReboot();
  }

  return (true);
}


//+++++++++++++++++++++++++++ Look for MQTT Broker +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
IPAddress SearchMQTTBroker()                   {  //Locate IP and Port

  IPAddress tryMqttBrkrIP = IPAddress {0, 0, 0, 0};

  DEBUG_ESP32_MQTT("T::88m: " + String(millis()) + "/n");

  Serial.println("    Looking for MQTT Broker broadcast IP");
  uint8_t nBrkrFound = MDNS.queryService("_mqtt", "tcp");
  if (nBrkrFound == 0) {               //no broker found
    delay (halfSec);
    nBrkrFound = MDNS.queryService("_mqtt", "tcp");      //try one more time
  }
  switch (nBrkrFound)       {        //See how many brokers found, if any
    case 0:
      Serial.println("     No broadcast.");
      break;
    case 1:    //one broker found
      {
        IPAddress firstIP = MDNS.IP(0);       //Check first IP
        if (firstIP[0] == 0)  {               //Just check first part of IP address
          Serial.println("!!  Broadcast Broker IP = null.");
        }
        else  {
          Serial.print("*** Broker IP broadcast: ");
          Serial.print(MDNS.hostname(0));
          Serial.print(" on IP ");
          Serial.print(firstIP);
          Serial.println("");
          tryMqttBrkrIP = firstIP;                //RETURN broker found
        }
        break;
      }
    default:                             //more than one found
      Serial.println(String (nBrkrFound) + " brokers found.  Using first found: ");
      for (uint8_t i = 0; i < nBrkrFound; ++i) {
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
      tryMqttBrkrIP = MDNS.IP(0);                //RETURN first IP in list
      break;
  }  //end of switch

  //Set up to see if broker found at this address
  return (tryMqttBrkrIP); //

}   //end of function


//++++++++++++++++ Do MQTT Subscriptions and initializations +++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool SubscribeMQTT ()                 {

  String fullTopic;
  bool mqPub[5];

  Serial.println("** MQTT Resets and Subscriptions");

  //Send MQTT initization messages to clear out prior retained messages
  fullTopic = mqttChannel + mqttWillTopic; fullTopic.remove(fullTopic.length() - 1, 1); //Strip off final slash
  mqPub[0] = mqttclient.publish(fullTopic.c_str(), "", true, 0);   //JMRI state: Retained = true; QoS = 0
  Serial.println("    Reset MQTT object: " + fullTopic);
  fullTopic = mqttChannel + mqttSubscribeTopics[SS_LT] + mqttClientID + JMRI_LT_ACT;          // ** NOTE: SubscribeTopics
  mqPub[1] = mqttclient.publish(fullTopic.c_str(), JMRITEST, true, 0);    //Light 0: JMRI set MC ready   ***FOR TESTING WITHOUT JMRI, JMRITEST set to: ON
  Serial.println("    Reset MQTT object: " + fullTopic);
  fullTopic = mqttChannel + mqttPublishTopics[SS_SENS] + mqttClientID + JMRI_SENS_ACK;        // ** NOTE: PublishTopics
  mqPub[2]  = mqttclient.publish(fullTopic.c_str(), "INACTIVE", true, 0);    //Retained = true; QoS = 0
  Serial.println("    Reset MQTT object: " + fullTopic);
  fullTopic = mqttChannel + mqttSubscribeTopics[SS_MEM] + mqttClientID + JMRI_MEM_ESP32_IP;   // ** NOTE: SubscribeTopics
  mqPub[3]  = mqttclient.publish(fullTopic.c_str(), "", true, 0);    //Memory: Retained = true; QoS = 0
  Serial.println("    Reset MQTT object: " + fullTopic);
  //Put IP address in JMRI memory for easy access
  fullTopic = mqttChannel + mqttPublishTopics[SS_MEM] + mqttClientID + JMRI_MEM_ESP32_IP;     // ** NOTE: PublishTopics
  mqPub[4] = mqttclient.publish(fullTopic, WiFi.localIP().toString(), true, 0);  //Retaned = true

  //Do required MQTT subscription after RESETs
  fullTopic = mqttChannel + "jStat/#";     //Status messages from JMRI
  mqttclient.subscribe(fullTopic);
  Serial.println("    Subscribing to " + fullTopic);
  for (uint8_t st = 0; st < 5; st++)    {
    if (mqttSubscribeTopics[st] != "")  {
      fullTopic = mqttChannel + mqttSubscribeTopics[st] + mqttClientID + "/#";
      Serial.println("    Subscribing to " + fullTopic);
      //bool subscribe(const char topic[], uint8_t qos);
      mqttclient.subscribe(fullTopic);
    }
  }

  return (mqPub[0] && mqPub[1] && mqPub[2] && mqPub[3] && mqPub[4]);
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ******************************************************************************************************
#endif   //END JMRI MQTT-specific functions
// ****************************************************************************************************
// *=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
