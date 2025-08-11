# MQTT Memory Communication
# Expecting MQTT broker to be available

# As of Version 60 (05/25/2023):
# Will subscribe to MQTT messages on topics "[JMRI channel]/jRecv/[MCid]/[mem name]" topic
#  and will update JMRI Memories with system name "IMM.[MCid]/[block]/nnn" or  "IMM.[MCid]/[mem name]"
# Will subscribe to "jRqst/[block]/mem" (request) for memory value and publish value
# Will publish to MQTT "jSend/[MemoryName]" when JMRI Memories with system name "IMQ.[MCid]/[something]" change

# Putting "RQ" in comment field of a "IMQ" memory will request the value for a "IMM" memory
# Putting "DISP" in comment field of a "IMQ" memory will request the value be displayed on the MC display
# Putting "REBOOT" first in memory contents and THEN in memory Comments of "IMQ" memory will send reboot message to microcontroller

# **  Must be coordinated with Arduino/ESP32 sketch to publish appropriate messages to topics: Sensors: Version 7c or later
# **  Must be coordinated with Arduino/ESP32 sketch to publish appropriate messages to topics: RFID Concentrator: Version 4d or later
# **  Use with coordinated JMRI Panel files: Version 51g or later

# Author: Jerry Grochow (c) 2020, 2021, 2022, 2023
# Software made available under The MIT License (https://opensource.org/licenses/MIT)
# Based on some code structures from Bob Jacobsen, Oscar Moutinho in the JMRI.org Jython Examples directory

import jmri
import java
from org.slf4j import LoggerFactory

DEBUG = False  #Additional level of debug info put in Script Output Window, to avoid unnecessary log calls
# *** To show debug messages in log, ALSO add the following line (without quotes) to the file 'default.lcf'
# *** located in the JMRI program directory: 'log4j.category.jmri.jmrit.jython.exec=DEBUG'
MQTTMemories_log = LoggerFactory.getLogger("jmri.jmrit.jython.exec.MQTTMemories")

#Update version number and date in following statement:
MQTTMemories_log.info("'MQTTMemories 64' version 01/17/2025 loaded")


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# For IMQ memory: parse MCid, object name, and block number (if any) from turnout or sensor or light system name
def MQTTMemories_getMQ(sysName):
    MCid     = None
    objName  = None
    blockNum = None
    _objName = sysName.split("Q.")             #Peel offprefix
    #if DEBUG:
    #    print "43 getMQ/Sysname: ", _objName
    if len(_objName) == 2:                     # Check for correctly formatted sysName
        _MCobjName = _objName[1].split("/")
        MCid = _MCobjName[0]
    #    if DEBUG:
    #       print ("48 getMQ/Sysname: ", _MCobjName)
        # If of form MCid/block/nnn...
        if len(_MCobjName) == 3:
           blockNum = str(_MCobjName[1])
           objName =  blockNum + "/" + str(_MCobjName[2])
        # If of form MCid/name, then blockNum will remain None
        elif len(_MCobjName) == 2:
           objName =  str(_MCobjName[1])
        else :
           print ("Unexpected Query Memory Name formet: ", _MobjName)
    #if DEBUG:
    #    print "46 getMQ/Sysname: ", MCid, " ", str(objName)
    return MCid, objName, blockNum

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# this is the code to be executed to publish a message destined to a microcontroller device
# "memory" for text messages
def MQTTMemories_Publish(topicSuffix, MCid, memName, payload):
    # set the retain option for transmissions (true is the JMRI default value)
    mqttAdapter.retained = False
    topic = topicSuffix + "/" + MCid
    if len(memName) > 0:
        topic = topic + "/" + memName         
    sent = mqttAdapter.publish(topic, payload)
    mqttAdapter.retained = True
    #Unusual event, so log as info
    MQTTMemories_log.info("'MQTTMemories' - " + memName + " SEND: [" + payload + "]")
    return sent

#=================================================================================
# listener class for incoming messages destined for JMRI Memories (IMM)
class MQTTMemories_MQTTListener(jmri.jmrix.mqtt.MqttEventListener) :
    def notifyMqttMessage(self, topic, message) :
        # Format: [JMRI channel]/jXxx/mem/[MCid]/[block]/nnn = 6 part topic
        # Or    : [JMRI channel]/jXxx/mem/[MCid]/[mem name]     = 5 part topic
        self.topicParts = topic.split("/")
        self.topicPartsNum = len(self.topicParts)
        if DEBUG:
            print "81 MQTTMemories_MQTTListener: ", topic, " ", message, self.topicPartsNum
            print self.topicParts
        # Format: [beginning of topic]/jXxx/mem/[microcontroller name]/[mem name] = 5 part message
        if (self.topicPartsNum == 5 and self.topicParts [self.topicPartsNum - 3]  == "mem"):           #Formatted by RFIDReader ESP32 sketch (no block number)
            self.memMCName = self.topicParts [self.topicPartsNum - 2]
            self.memName    = str(self.topicParts [self.topicPartsNum - 1])           
            self.memSystemName = "IMM." + self.memMCName + "/" + self.memName
            if DEBUG:
                print "89 Memory name: ", self.memMCName, " ", self.memName
            #See if memory exists; if not, create it and also corresponding "Q" memory (for user to send messages to MC)
            self.myMemory = memories.getBySystemName(self.memSystemName)
            if (self.myMemory == None):
                #If memories being created (unusual), log to Script Output and System Console            
                self.myMemory = memories.provideMemory(self.memSystemName)
                if DEBUG:
                    print ("95 Created memory: " + self.memSystemName)
                MQTTMemories_log.info("'MQTTMemories - Created memory:'  - " +  self.memSystemName)
                #Find/create corresponding "query" memory (to be used to send messages to MC)
                self.memSystemNameQ = "Q." + self.memMCName + "/" + self.memName
                self.myMemoryQ = memories.provideMemory(self.memSystemNameQ)
                if DEBUG:
                    print ("100 Created memory: " + self.memSystemNameQ)
                MQTTMemories_log.info("'MQTTMemories - Created Qmemory:' - " +  self.memSystemNameQ)
                self.myMemoryQ.setComment("[RQ: request; DISP: display on MC; REBOOT: reboot MC]")
                #Start listening for user messages put in IMQ memory
                self.myMemoryQ.addPropertyChangeListener(MQTTMemories_MemoryQListener())
                print  "Memory Listener started: ", self.memSystemNameQ 
            self.myMemory.value = message
            self.myMemory.comment = "Via MQTT"    
        # Format: [beginning of topic]/jXxx/mem/[MCid]/block/nnn = 6 part message
        elif (self.topicPartsNum == 6  and self.topicParts [self.topicPartsNum - 4]  == "mem"):     #Formatted by Multisensor ESP32 sketch
            self.memMCName = self.topicParts [self.topicPartsNum - 3]
            self.blockNum  = str(self.topicParts [self.topicPartsNum - 2])
            self.memName    = self.blockNum + "/" + str(self.topicParts [self.topicPartsNum - 1])           
            self.memSystemName = "IMM." + self.memMCName + "/" + self.memName
            if DEBUG:
                print "115 Memory name: ", self.memMCName, " ", self.memName
            #See if memory exists; if not, create it and corresponding "Q" memory for output
            self.myMemory = memories.getBySystemName(self.memSystemName)
            if (self.myMemory == None):                     
                #If memories being created (unusual), log to Script Output and System Console            
                self.myMemory = memories.provideMemory(self.memSystemName)
                print ("121 Created memory: " + self.memSystemName)
                MQTTMemories_log.info("'MQTTMemories - Created memory:'  - " +  self.memSystemName)
                #Find/create corresponding "query" memory (to be used to send messages to MC)
                self.memSystemNameQ = "Q." + self.memMCName + "/" + self.memName
                self.myMemoryQ = memories.provideMemory(self.memSystemNameQ)
                print ("126 Created memory: " + self.memSystemNameQ)
                MQTTMemories_log.info("'MQTTMemories - Created Qmemory:' - " +  self.memSystemNameQ)
                self.myMemoryQ.setComment("[RQ: request; DISP: display on MC; REBOOT: reboot MC]")
                #Start listening for user messages put in IMQ memory
                self.myMemoryQ.addPropertyChangeListener(MQTTMemories_MemoryQListener())
                print  "Memory Listener started: ", self.memSystemNameQ 
            self.myMemory.value = message
            self.myMemory.comment = "Via MQTT"    
        # If can't parse...   
        else:
            print "Unexpected topic: ", topic, " Msg:" + message
            MQTTMemories_log.warn("'MQTTMemories' - Unexpected topic: " + topic + " Msg:" + message)
#             (+ " [property=" + event.propertyName + ": oldValue=" + str(event.oldValue) + " newValue=" + str(event.newValue) + "]")             
        return

#=================================================================================
# Define the listener class for "IMQ" Memories to send out when changed
# "RQ" in value or comment will send request for memory contents (to be put into "IMM" by microcontroller sketch)
class MQTTMemories_MemoryQListener(java.beans.PropertyChangeListener):    
    def propertyChange(self, event):
        self.myMemory = event.getSource()
        self.myMemoryName = self.myMemory.getDisplayName(jmri.NamedBean.DisplayOptions.SYSTEMNAME)
        MQTTMemories_log.debug("'MQTTMemories' - Memory=" + self.myMemoryName + " property=" + event.propertyName + "]: oldValue=" + str(event.oldValue) + " newValue=" + str(event.newValue))
        if DEBUG:
            print "148 MemoryListener:  ", self.myMemoryName, " ", event.propertyName, " ", event.newValue
        self.MCid, self.memName, self.blockNum = MQTTMemories_getMQ(self.myMemory.getSystemName())
        sent = False
        # Commands to do something put in comments field operate on value (change value first)
        # print "159: ", event.propertyName," : ", self.myMemory.getComment()
        if event.propertyName == "Comment":   #Request value else ignore       
            if len(self.myMemory.getComment().split("Sent")) == 1:
                if self.myMemory.getComment() == "RQ":  # Request memory value from device
                    sent = MQTTMemories_Publish("jRqst/mem", self.MCid, self.memName, "")
                    self.myMemory.setComment("Sent RQ")     # Reset request
                elif self.myMemory.getComment() == "DISP":  # Request memory value be display on device
                    sent = MQTTMemories_Publish("jDisp/mem", self.MCid, self.memName, self.myMemory.getValue())
                    self.myMemory.setComment("Sent DISP")     # Reset request
                elif self.myMemory.getComment() == "REBOOT" and self.myMemory.getValue() == "REBOOT":  # Request microcontroller reboot itself
                    print "REBOOT Sent to: ", self.MCid
                    MQTTMemories_log.warn("'MQTTMemories' - REBOOT Sent to: " + self.MCid)
                    sent = MQTTMemories_Publish("jRbot/REBOOT", self.MCid, "", self.myMemory.getValue())
                    self.myMemory.setComment("Sent REBOOT")     # Reset request
                else:
                    print "Unexpected Comment Command from: ", self.myMemoryName, " : ", str(event.newValue)
                    MQTTMemories_log.warn("'MQTTMemories' - Unexpected Comment Command from QMemory=" + self.myMemoryName + " property=" + event.propertyName + "]: oldValue=" + str(event.oldValue) + " newValue=" + str(event.newValue)) 
            else:
                return            
        elif event.propertyName == "value":    # Value changed either by JMRI or on input from device 
            return        
        else:
            print "Unexpected PropChng for" + self.myMemoryName, " [property=", event.propertyName, "]: oldValue=", str(event.oldValue), " newValue=", str(event.newValue)
            MQTTMemories_log.warn("'MQTTMemories' - Unexpected PropChng for Memory=" + self.myMemoryName + " [property=" + event.propertyName + "]: oldValue=" + str(event.oldValue) + " newValue=" + str(event.newValue))             
        return



#----- MAINLINE --------------------------------------------------------------------------------
if globals().get("MQTTMemories_running") != None: # Script already loaded so exit script
    MQTTMemories_log.warn("'MQTTMemories' already loaded and running. Restart JMRI before loading this script.")
else: # Continue running script
    MQTTMemories_log.info("'MQTTMemories' started.")
    print "IMQ: Comments: RQ (to have MC publish mem contents, DISP (to display mem contents)"
    print "               REBOOT first in mem contents and THEN in Comments (to reboot microcontroller)"
    MQTTMemories_running = True               #So script won't be loaded twice

    #Register memory topic
    topic = "jRecv/mem/#"  # this will be prefixed automatically by JMRI channel, e.g. "trains/"
    #Find the MqttAdapter
    mqttAdapter = jmri.InstanceManager.getDefault( jmri.jmrix.mqtt.MqttSystemConnectionMemo ).getMqttAdapter()    
    #Subscribe to MQTT messages to memories topics
    mqttAdapter.subscribe(topic, MQTTMemories_MQTTListener())
    print  "MQTT Topic Subscribed, Listener set: ", topic
    
    #Set up listeners for changes to JMRI "MQ" memories that already exsit to capture and send messages to the corresponding MC
    for myMemory in memories.getNamedBeanSet():
        MCid, memName, blockNum = MQTTMemories_getMQ(myMemory.getSystemName())
        if (MCid != None and memName != None):            #Only for memories named in expected format
            MQTTMemories_log.debug("'MQTTMemories' - Memory SystemName Accepted [" + myMemory.getSystemName() + "] MCid [" + MCid + "] memName [" + str(memName) + "]")
            myMemory.setValue("")
            myMemory.setComment("[RQ: request; DISP: display on MC; REBOOT: reboot MC]")
            myMemory.addPropertyChangeListener(MQTTMemories_MemoryQListener())
            print  "Memory Listener set: ", myMemory
    
    #Create and set sensor to indicate this script run successfully
    sensors.provideSensor("IS.MQ$LSTNR").setState(jmri.Sensor.ACTIVE)  #LISTENERS ACTIVE - could be used to trigger a Logix
    
# Description of script from Tom Seitz 2023-05-29    
# The script begins with imports and logging setup, including a debug flag for additional information.
# The MQTTMemories_getMQ() function is defined to parse system names into meaningful components. It is used in later parts of the script to analyze the name of memory objects.
# MQTTMemories_Publish() function is defined for publishing MQTT messages to microcontroller devices.
# MQTTMemories_MQTTListener class is defined to handle inbound MQTT messages. It analyzes incoming MQTT topics and updates JMRI memory objects accordingly.
# MQTTMemories_MemoryQListener class is defined to handle changes in JMRI memory objects. It listens for changes in certain JMRI memory objects and publishes these changes to MQTT topics.
# In the mainline part of the script, if the script is not already running, it subscribes to a specified MQTT topic and sets up listeners for JMRI memory objects.

# In this script, the MQTT topics being used are:

# [JMRI channel]/jRecv/[MCid]/[mem name]: This is a topic that the script subscribes to, where JMRI channel is a prefix automatically added by JMRI (like "trains/"), and MCid and mem name will be variables in the script. When a message is published to this topic by an external source (like a microcontroller), the script will receive it and update JMRI Memories.

# jRqst/[block]/mem: This is a topic the script uses to request a memory value. When a request is sent on this topic, it expects the external device to respond with the memory value.

# jSend/[MemoryName]: This is a topic that the script publishes to when a JMRI Memory with a specific system name changes. The MemoryName will be replaced by the name of the memory that changed.

# jDisp/mem: Used to request a memory value to be displayed on a device.

# jRbot/mem: Used to send a reboot command to a microcontroller device.
         
