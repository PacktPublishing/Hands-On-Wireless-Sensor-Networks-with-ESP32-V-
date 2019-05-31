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

//An object structure presenting node name and ID
typedef struct {
  String id;
  String name;
} NodeObject;

//An object structure presenting node name and its neighbours (max 10)
struct Neighbours{
    String nodeName;
    String neighbourList[10];
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

//                                            node | list of neighbours
struct Neighbours allNodesNeighboursList[] = { "a",  { "b", "c" },
                                               "b",  { "a", "d", "e" },
                                               "c",  { "a", "d", "f" },
                                               "d",  { "b", "c", "e" },
                                               "e",  { "b", "d", "f" },
                                               "f",  { "e", "c" }, 
                                             };

String thisNodeName = "";    //string variable to store current node name
String GATEWAY_NODE = "D";   //Node from where commands will be propogated into the network by user
String neighbours[10] = {};
int totalNeighbours = 0; 

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
int line = 10;               // define oLed line space
boolean overwrite = false;   // flag to indicate last line of oLed, based on which we override existing characters from screen
String reportString = "";    // string to hold Serial bytes
String rcvdMsgCrc[100] = {}; // string array to store last 100 crc
int crcStoreIndex = 0;       // integer referring to store crc at apecific location of array

// `message`: A Json packet structure that is sent across nodes to transfer data
// msg: a json containing following two items; 
//  -- data: actual content of the message
//  -- time: epoch time when this message was created
// crc: checksum of the above message 
const char message[] = "{\"msg\": {\"data\":\"\", \"time\": }, \"crc\": }";

// User stub
void broadcastMessage(String msg) ; // Prototype so PlatformIO doesn't complain

void broadcastMessage(String msg) {
  pout("TX >> " + msg);
  mesh.sendBroadcast( msg );
}

void sentMessage(uint32_t dest, String msg) {
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

void sentToAllNeighbours(String msg) {
    //iterate through all nodes neighbour list
    for (uint8_t i = 0; i < sizeof(allNodesNeighboursList) / sizeof(Neighbours); ++i) {
       //if this node name matches node Name in the list
       if (thisNodeName.equalsIgnoreCase(allNodesNeighboursList[i].nodeName)) {
          //iterate through all neighbours of this node
          for (uint8_t j = 0; j < sizeof(allNodesNeighboursList[i].neighbourList)/12; ++j) {
              //get each neighbour node id
              uint32_t nodeId = getNodeIdByName(allNodesNeighboursList[i].neighbourList[j]);
              //no point of sending msg to unknown node
              if (nodeId != 0) {    
                  Serial.print("> sending msg to neighbour ");
                  Serial.println(allNodesNeighboursList[i].neighbourList[j]);
                  sentMessage(nodeId, msg);
              }
          }
       }
    }
}

void sentToAllNeighboursExceptOrigin(String msg, uint32_t origin) {
    //iterate through all nodes neighbour list 
    for (uint8_t i = 0; i < sizeof(allNodesNeighboursList) / sizeof(Neighbours); ++i) {
       //if this node name matches node Name in the list
       if (thisNodeName.equalsIgnoreCase(allNodesNeighboursList[i].nodeName)) {
          //iterate through all neighbours of this node
          for (uint8_t j = 0; j < sizeof(allNodesNeighboursList[i].neighbourList)/12; ++j) {
              //get each neighbour node id
              uint32_t nodeId = getNodeIdByName(allNodesNeighboursList[i].neighbourList[j]);
              //don't sent msg to unknown node and origin node
              if (nodeId != 0 && nodeId != origin) {
                  Serial.print("> sending msg to neighbour except origin " + getNodeNameById(origin));
                  Serial.println(allNodesNeighboursList[i].neighbourList[j]);
                  sentMessage(nodeId, msg);
              }
          }
       }
    }
}


void ProcessMeshMessage(uint32_t from, String recvMessage){
  DynamicJsonBuffer jsonDynBufRx;
  JsonObject& recvMessageJson = jsonDynBufRx.parseObject(recvMessage); 
  String recvMsgFrom = getNodeNameById(from);
  String recvMsgData = recvMessageJson["msg"]["data"].as<String>();
  String recvMsgDataCrc = recvMessageJson["crc"].as<String>();

  Serial.println("RX from  : " + recvMsgFrom);
  Serial.println("  -- data: " + recvMsgData);
  Serial.println("  -- crc : " + recvMsgDataCrc);
      
  if (validateRxMessage(recvMessageJson)) {   //check if this msg not already received
      storeRxMsgCrc(recvMsgDataCrc);
      pout("RX from " + recvMsgFrom + " - " + recvMsgData);   
      sentToAllNeighboursExceptOrigin(recvMessage, from);
   }
}


void storeRxMsgCrc(String crc) {
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
    String receivedCrc = rxMsg["crc"].as<String>();
    String calculatedCrc = calculateCrc(rxMsg["msg"]);
    if (receivedCrc.equals(calculatedCrc)) {
        if (isDuplicateMsg(receivedCrc)) {
            pout("DUP msg " + receivedData + " !!");   
            Serial.println("duplicate message received. ignoring..."); 
            return false;
        } else { 
           Serial.println("Message validation successful"); 
           return true;
        }
    } else {
        Serial.println("CRC did not match.");   
        return false;
    }
}

String calculateCrc(JsonObject& msgData) {
    CRC16 crc;
    char buffer[500];
    msgData.printTo(buffer, sizeof(buffer));
    crc.processBuffer(buffer, strlen(buffer)); 
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
  InitOLed();
  
  Serial.println("This nodes neighbours are: ");
  for (uint8_t i = 0; i < sizeof(allNodesNeighboursList) / sizeof(Neighbours); ++i) {
     if (thisNodeName.equalsIgnoreCase(allNodesNeighboursList[i].nodeName)) {
        for (uint8_t j = 0; j < sizeof(allNodesNeighboursList[i].neighbourList)/12; ++j) {
          if (allNodesNeighboursList[i].neighbourList[j] != "" ) {
              Serial.print("> ");
              Serial.println(allNodesNeighboursList[i].neighbourList[j]);            
          }
        }
     }
  }

}

void InitOLed() {
  String initMsg = "Node " + thisNodeName + " (id: " + String(mesh.getNodeId()) + ")";
  display.init();
  display.flipScreenVertically();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, initMsg);
  display.display();
}

void SetupMeshNetwork() {
  //mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); // all types on
  mesh.setDebugMsgTypes( ERROR | STARTUP | CONNECTION );  // set before init() so that you can see startup messages
  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT );
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
}

JsonObject& createJsonMsg(String msgData) {
    //create a json msg structure; 
    //"{\"msg\": {\"data\":\"\", \"time\": }, \"crc\": }"; 
    DynamicJsonBuffer jsonDynBuf;
    JsonObject& jsonObj = jsonDynBuf.parseObject(message);   
    jsonObj["msg"]["data"] = msgData;
    jsonObj["msg"]["time"] = String(millis());
    jsonObj["crc"] = calculateCrc(jsonObj["msg"]); 
    return jsonObj;
}

void loop() {

  //User inputs from a Gateway node only. 
  if (thisNodeName.equalsIgnoreCase(GATEWAY_NODE)) {
    while (Serial.available()) {
      char inChar = (char)Serial.read();
      reportString += inChar;  //collect every byte until end of line
      if (inChar == '\n') {    //new line character
          String trimmedMsg = reportString.substring(0, reportString.length()-1); //remove new line `\n` char from string.
          if (trimmedMsg.startsWith("F:")) {
              String msgToSent = trimmedMsg.substring(2, trimmedMsg.length());
              pout("Floading msg: `" + msgToSent + "`");
              JsonObject& jsonMsg = createJsonMsg(msgToSent);

              //store the msg crc into nodes memory - to avoid receiving this msg again. 
              storeRxMsgCrc(jsonMsg["crc"].as<String>());

              //convert jsonMsg to String
              String jsonMsgStr = "";
              jsonMsg.printTo(jsonMsgStr);  //convert json to string and store it in `out`
              Serial.println("Sending msg: " + jsonMsgStr);  

              //sent to all neighbours             
              sentToAllNeighbours(jsonMsgStr);
          } else {
              // sent msg to invididual node by name (testing purpose only)
              uint32_t destNode = getNodeIdByName(trimmedMsg);
              if (destNode != 0) {
                  sentMessage(destNode, "Hello from " + thisNodeName);
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
    line = 10;
  }
}
