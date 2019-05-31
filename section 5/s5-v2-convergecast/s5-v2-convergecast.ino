//************************************************************
//
// Packt> Wireless Sensors Network using ESP32
// Section 5 - video 1
// Basic code for nodes to communicate via PainlessMesh protocol
//
//************************************************************

#include "crc16.h"     //https://github.com/jpralves/crc16
#include "painlessMesh.h"
#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306Wire.h" // legacy include: `#include "SSD1306.h"`

#define   MESH_PREFIX     "mymeshnetwork"
#define   MESH_PASSWORD   "12345678"
#define   MESH_PORT       5555

//An object structure presending node name and ID
typedef struct {
  String id;
  String name;
} NodeObject;

//An object structure presending parent node name and its child nodes (max 10)
struct SpanningTree {
    String parentNode;
    String childNodes[10];
};

///////////////////////////////////////
//////// pin configuration
///////////////////////////////////////
#if defined(ESP8266)
#define SDA_PIN D2
#define SCL_PIN D1
const int LED_RED =  D0;
const int LED_YELLOW =  D5;
const int LED_GREEN =  D6;
#elif defined(ESP32)
#define SDA_PIN 22
#define SCL_PIN 21
const int LED_RED =  15;
const int LED_YELLOW =  16;
const int LED_GREEN =  17;
#endif

Scheduler userScheduler; // to control your personal task
painlessMesh  mesh;

//                                            ParentNode | childNodes
struct SpanningTree allParentAndChildrenList[] = { "a", { "c", "f" },
                                                   "b", { "e", "a" },
                                                   "c", { },
                                                   "d", { "b" },
                                                   "e", { },
                                                   "f", { }, 
                                                 };
                                                 
//                                                                               
String childrenResponse[10] =  {};    //list of childNode messages who responded - initially 0     
String childNodes[10] = {};           //hold the names of all child nodes
int totalChildNodes = 0;              //total number of children
int totalChildResponseReceived = 0;   //integer to count how many children have sent the response back
String parentNode = "";               //holds the parent node name
String thisNodeName = "";             //string variable to store current node name
String ROOT_NODE = "D";            //Node from where commands will be propogated into the network by user

const NodeObject nodeNameIdRecord[] {
  {"2382599453", "A"},
  {"986000249",  "B"},
  {"3950682377", "C"},  //esp8266
  {"2382600661", "D"},
  {"2382599661", "E"}, 
  {"3950700603", "F"}   //esp8266
};

SSD1306Wire  display(0x3c, SDA_PIN, SCL_PIN);

//helper variables           -- do not change!
int temperature = 0;         // temperature value. currently not taking from actual sensor, but generating random value in setup()
int line = 10;               // define oLed line space
boolean overwrite = false;   // flag to indicate last line of oLed, based on which we override existing characters from screen
String reportString = "";    // string to hold Serial bytes
String rcvdMsgCrc[100] = {}; // string array to store last 100 crc
int crcStoreIndex = 0;       // integer referring to store crc at apecific location of array

// `message`: A Json packet structure that is send across nodes to transfer data
// msg: a json containing following two items; 
//  -- data: actual content of the message
//  -- type: req  - request directing from root node to leaf nodes (flooding)
//         \ resp - response directing from leaf nodes to root node (convergecast)             
//  -- time: epoch time when this message was created
// crc: checksum of the above message 
// 
const char message[] = "{\"msg\": {\"type\":\"\", \"data\":\"\", \"time\": }, \"crc\": }";
// Depricated - due to PainlessMesh protocol message size limitation, instead message will be sent in following format;
// <type>-<data>,<time>#<crc>
// e.g.
// "req-getMin,17263#1261"


// User stub
void broadcastMessage(String msg) ; // Prototype so PlatformIO doesn't complain

void broadcastMessage(String msg) {
  pout("TX >> " + msg);
  mesh.sendBroadcast( msg );
}

void sendMessage(uint32_t dest, String msg) {
  mesh.sendSingle(dest, msg);
}

String getNodeNameById(uint32_t id) {
  String strId = String(id);
  for (uint8_t i = 0; i < sizeof(nodeNameIdRecord) / sizeof(NodeObject); ++i) {
    if (strId.equals(nodeNameIdRecord[i].id)) {
        return nodeNameIdRecord[i].name;
    }
  }
  return String("--");
}

uint32_t getNodeIdByName(String nodeName) {
  for (uint8_t i = 0; i < sizeof(nodeNameIdRecord) / sizeof(NodeObject); ++i) {
    if (nodeName.equalsIgnoreCase(nodeNameIdRecord[i].name)) {
        String strId = nodeNameIdRecord[i].id;
        uint32_t id = strtoul(strId.c_str(), NULL, 10);
        return id;
    }
  }
  Serial.println("Unknown node name " + nodeName);
  return 0;
}

void sendToAllChildren(String msg) {
    for (uint8_t i = 0; i < totalChildNodes; ++i) {
        uint32_t nodeId = getNodeIdByName(childNodes[i]);
        //no point of sending msg to unknown node
        if (nodeId != 0) {    
            Serial.print("> sending msg to child " + String(nodeId) + " - ");
            Serial.println(childNodes[i]);
            sendMessage(nodeId, msg);
        } else {
            Serial.println("> unknown node found while sending to child " + String(childNodes[i]));                          
        }
    }    
}

void sendToParent(String msg) {
    Serial.println("> sending msg to parent " + parentNode);
    sendMessage(getNodeIdByName(parentNode), msg);        
}

void ProcessMeshMessage(uint32_t from, String recvMessage){
  //////////////////////////////////////////////////////////////////
  //---------------------Decode received message
  //We receive msg in this form "req-getMin,17263#1261"
  //Where; 
  //  --  type = req
  //  --  data = getMin
  //  --  time = 17263
  //  --  crc  = 1261
  int dashIndex = recvMessage.indexOf('-');
  int commaIndex = recvMessage.indexOf(','); 
  int hashIndex = recvMessage.indexOf('#');
  String recvMsgFrom = getNodeNameById(from);
  String recvMsgType = recvMessage.substring(0, dashIndex);
  String recvMsgData = recvMessage.substring(dashIndex + 1, commaIndex);
  String recvMsgTime = recvMessage.substring(commaIndex + 1, hashIndex);
  String recvMsgCrc = recvMessage.substring(hashIndex + 1, recvMessage.length()); 
  //////////////////////////////////////////////////////////////////     
  
  DynamicJsonBuffer jsonDynBuf;
  JsonObject& recvMessageJson = jsonDynBuf.parseObject(message);  
  recvMessageJson["msg"]["type"] = recvMsgType; 
  recvMessageJson["msg"]["data"] = recvMsgData;
  recvMessageJson["msg"]["time"] = recvMsgTime;
  recvMessageJson["crc"] = recvMsgCrc; 

  Serial.println("RX from  : " + recvMsgFrom);
  Serial.println("  -- data: " + recvMsgData);
  Serial.println("  -- type: " + recvMsgType);  
  Serial.println("  -- crc : " + recvMsgCrc);
      
  if (validateRxMessage(recvMessageJson)) {   //check if this msg not already received
      storeMsgCrc(recvMsgCrc);
      pout(recvMsgType + " from " + recvMsgFrom + " - " + recvMsgData);  
      if (recvMsgType.equalsIgnoreCase("req")) {
          if (isLeafNode(thisNodeName)) {
             //the request has finally reached leaf node
             //send value "RESP" to its parent
             pout("Resp: value " + String(temperature) + "°C");
             String shortMsg = createShortMsg("resp", String(temperature));
             Serial.println("Sending msg: " + shortMsg);  
             //send value to parent node            
             sendToParent(shortMsg);
          } else { 
             //propogate msg to children until it reaches leaf nodes
             sendToAllChildren(recvMessage);
          }
      } else if (recvMsgType.equalsIgnoreCase("resp") && (!isLeafNode(thisNodeName))) {
          totalChildResponseReceived++;
          childrenResponse[totalChildResponseReceived-1] = recvMsgFrom + "-" + recvMsgData;  //store <nodeName>-<value>        
          Serial.println("Total Child Response Received: " + String(totalChildResponseReceived) + " (out of " + String(totalChildNodes) + ")");
          if (totalChildResponseReceived == totalChildNodes) { //received response from all children
              int nodesValue[totalChildNodes + 1]; //holds value of all child nodes + its own value
              nodesValue[0] = temperature; //first add its own temperature value on 0th index
              Serial.print("Children resp: ");
              for (uint8_t i = 0; i < totalChildResponseReceived; ++i) {
                   Serial.print(childrenResponse[i] + ", ");
                   int value = (childrenResponse[i].substring(2, childrenResponse[i].length())).toInt();  //extract value 
                   nodesValue[i+1] = value;      
              }
              Serial.println("");
              totalChildResponseReceived = 0;
              int minTemperature = 999;
              for ( int i = 0; i < sizeof(nodesValue)/sizeof(nodesValue[0]); i++ ){   
                if ( nodesValue[i] < minTemperature ) {
                     minTemperature = nodesValue[i];
                }
              }                
              pout("Min temp " + String(minTemperature) + "°C");
              String shortMsg = createShortMsg("resp", String(minTemperature)); 
              if (parentNode != "") {
                  sendMessage(getNodeIdByName(parentNode), shortMsg); 
              } else {
                 //this is the root node
                 Serial.println("The final minimun value is: " + String(minTemperature));
              }                                                 
          }
      } 
   }
}

boolean isLeafNode(String nodeName) {
    if (totalChildNodes == 0) {
       return true;
    } else {
       return false;
    }
}

void storeMsgCrc(String crc) {
    //store the given crc into internal cache memory
    rcvdMsgCrc[crcStoreIndex] = crc;
    crcStoreIndex += 1;
    if (crcStoreIndex > 100) {
        //flush existing crc's if memory reaches 100
        crcStoreIndex = 0;
    }
}

boolean isDuplicateMsg(String crc){ 
   //iterate through all stored crc's 
   for (int i = 0; i < 100; i++) {
        //check if given crc is already stored in cache memory
        if (rcvdMsgCrc[i].equals(crc)) {
            return true;
        }
      }  
    return false;
}    

boolean validateRxMessage(JsonObject& rxMsg) {
    String receivedData = rxMsg["msg"]["data"].as<String>();
    String receivedTime = rxMsg["msg"]["time"].as<String>();      
    String receivedCrc = rxMsg["crc"].as<String>();
    String calculatedCrc = calculateCrc(receivedData + "," + receivedTime);
    if (receivedCrc.equals(calculatedCrc)) {
        if (isDuplicateMsg(receivedCrc)) {
            pout("DUP msg " + receivedData + " !!");   
            Serial.println("duplicate message received. ignoring..."); 
            return false;
        } else { 
           //Serial.println("Message validation successful"); 
           return true;
        }
    } else {
        Serial.println("CRC did not match.");   
        return false;
    }
}

String calculateCrc(String msgData) {
    CRC16 crc;
    crc.processBuffer(msgData.c_str(), strlen(msgData.c_str())); 
    return String(crc.getCrc()); 
}


// Needed for painless library
void receivedCallback(uint32_t from, String &msg ) {
  Serial.printf("Received from %u msg=%s\n", from, msg.c_str());
  ProcessMeshMessage(from, msg);
}

void newConnectionCallback(uint32_t nodeId) {
  //triggered when new node joins network
  Serial.printf("New Connection, nodeId = %u\n", nodeId);
}

void changedConnectionCallback() {
  //triggered when connection change e.g. node exiting the network
  Serial.printf("Changed connections %s\n", mesh.subConnectionJson().c_str());
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  SetupMeshNetwork();

  //initializing painlessmesh network drains more current,
  //so better to turn on oLed a few seconds after it finishes
  delay(2000);

  thisNodeName = getNodeNameById(mesh.getNodeId());
  temperature = random(15,30); //generate a random integer value between 15 to 30. 
                               //This will represent a dummy temperature value, which (in actual) could be taken from real sensor
  
  InitOLed();
  displayNodeInfo();  // method will populate this nodes children and parent.

}

void displayNodeInfo(){
  Serial.print("Node name: " + thisNodeName);
  //iterate through allParentAndChildrenList list to find its children nodes
  for (uint8_t i = 0; i < sizeof(allParentAndChildrenList) / sizeof(SpanningTree); ++i) {
     String parentNodeName = allParentAndChildrenList[i].parentNode;
     if (thisNodeName.equalsIgnoreCase(parentNodeName)) {
          for (uint8_t j = 0; j < sizeof(allParentAndChildrenList[i].childNodes)/12; ++j) {
            if (allParentAndChildrenList[i].childNodes[j] != "" ) {
                totalChildNodes++;
                childNodes[j] = allParentAndChildrenList[i].childNodes[j];        
            }
          }
          if (totalChildNodes > 0) {
              Serial.print(", with " + String(totalChildNodes) + " children; ");  
              for (uint8_t i = 0; i < totalChildNodes; ++i) { 
                 Serial.print(childNodes[i] + ", ");                             
              }
          } else {
              Serial.println(" (leaf node)");
          }
     }
  }
  Serial.println("");
  
   //iterate through allParentAndChildrenList list to find its parent node
   for (uint8_t i = 0; i < sizeof(allParentAndChildrenList) / sizeof(SpanningTree); ++i) {
      String parentNodeName = allParentAndChildrenList[i].parentNode;
      for (uint8_t j = 0; j < sizeof(allParentAndChildrenList[i].childNodes)/12; ++j) {
         String childNodeName = allParentAndChildrenList[i].childNodes[j];
         if (thisNodeName.equalsIgnoreCase(childNodeName)) {
             parentNode = parentNodeName;
         }
      }
   }
   
   if (parentNode == "") {
      Serial.println("No parent node found for this node. Hence its Root node.");    
   } else {
      Serial.println("Parent node: " + parentNode);      
   }
  
}

void InitOLed() {
  String initMsg = "Node " + thisNodeName + " (id: " + String(mesh.getNodeId()) + ")";
  display.init();
  display.flipScreenVertically();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, initMsg);
  display.drawString(0, line, "Value: " + String(temperature) + "°C");
  display.display();
  line += 10;
}

void SetupMeshNetwork() {
  //mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); // all types on
  mesh.setDebugMsgTypes( ERROR | STARTUP | CONNECTION );  // set before init() so that you can see startup messages
  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT );
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
}

String createShortMsg(String type, String msgData) {
    //PainlessMesh behaves strange when sending long JSON messages. So we are going to create a short message
    //by seperating each field with special characters. e.g. 
    //<type>-<data>,<time>#<crc>
    //e.g. 
    //req-getMin,17263#1261
    String msgTime = String(millis());
    String calcCrc = calculateCrc(msgData + "," + msgTime);

    //store the msg crc into nodes memory - to avoid receiving this msg again. 
    storeMsgCrc(calcCrc);

    String msg = type + "-" + msgData + "," + msgTime + "#" + calcCrc; 
    return msg;
}

void loop() {

  //User inputs from a Gateway node only. 
  if (thisNodeName.equalsIgnoreCase(ROOT_NODE)) {
    while (Serial.available()) {
      char inChar = (char)Serial.read();
      reportString += inChar;  //collect every byte until end of line
      if (inChar == '\n') {    //new line character
          String trimmedMsg = reportString.substring(0, reportString.length()-1); //remove new line `\n` char from string.
          if (trimmedMsg.startsWith("R:getMin")) {
              String msgTosend = trimmedMsg.substring(2, trimmedMsg.length());
              pout("Requesting: `" + msgTosend + "`");
              String shortMsg = createShortMsg("req", msgTosend);
              Serial.println("Sending msg: " + shortMsg);  
              //send to all Children             
              sendToAllChildren(shortMsg);
          } else {
              // send msg to invididual node by name (testing purpose only)
              uint32_t destNode = getNodeIdByName(trimmedMsg);
              if (destNode != 0) {
                  sendMessage(destNode, "Hello from " + thisNodeName);
              }            
          }
        reportString = "";   //empty the string
      }
    }
  }
  userScheduler.execute(); // it will run mesh scheduler as well
  mesh.update();
}

//helper method for oLed
//pout -> Print Out
void pout(String msg) {
  if (overwrite == true) {
    display.setColor(BLACK);
    display.fillRect(0, line, 128, 64);
    display.setColor(WHITE);
    display.drawString(0, line, msg);
    display.display();
    line += 10;
  } else {
    display.drawString(0, line, msg);
    display.display();
    line += 10;
  }

  if (line == 60) {
    overwrite = true;
    line = 20;
  }
}
