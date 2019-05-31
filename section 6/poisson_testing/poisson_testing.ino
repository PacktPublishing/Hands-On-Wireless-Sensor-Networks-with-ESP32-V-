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

//An object structure presenting node name and ID
typedef struct {
  String nodeState;
  int statePinNumber;
} NodeStatePin;

///////////////////////////////////////
//////// pin configuration
///////////////////////////////////////
#if defined(ESP8266)
#define SDA_PIN D2
#define SCL_PIN D1
const int LED_RED =  D8;
const int LED_YELLOW =  D7;
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
struct Neighbours allNodesNeighboursList[] = { "a",  { "d", "c", "d", "e", "f" },
                                               "b",  { "a", "c", "d", "e", "f" },
                                               "c",  { "a", "d", "d", "e", "f" },
                                               "d",  { "a", "d", "c", "e", "f" },
                                               "e",  { "a", "d", "c", "d", "f" },
                                               "f",  { "a", "d", "c", "d", "e" }, 
                                             };

String thisNodeName = "";    //string variable to store current node name
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

//Node state variables
long randNumber; 
String currentNodeState = ""; //holds the current node state Red/Yellow/Green
NodeStatePin allPossibleStates[3] = {  {"Red", LED_RED}, 
                                       {"Yellow", LED_YELLOW},
                                       {"Green", LED_GREEN}
                                    };

//poisson process variables
float rate_parameter = 1.0 / 10.0;
int nextTriggerCounter = 0;
float nextTrigger = 0;              // next iteration of 
const long iteration_time = 500;  // after every iteration_time(millis), nextTriggerCounter++ to trigger poisson process
boolean activeMode = false;       // true: if ndoe is waiting for `resp`, else `false`
int time_out_constant = 5000;     // total time to wait for pair response. after this time, set activeMode=false
unsigned long startTime;          // time at which the pair request is sent
String reqNodeName = "";          // holds the name of node to which the current request was sent

//helper variables              -- do not change!
unsigned long currentTime;        // variable to hold current time
unsigned long previousMillis = 0; //variable to hold last action time   
int line = 10;                    // define oLed line space
boolean overwrite = false;        // flag to indicate last line of oLed, based on which we override existing characters from screen
String reportString = "";         // string to hold Serial bytes
String rcvdMsgCrc[100] = {};      // string array to store last 100 crc
int crcStoreIndex = 0;            // integer referring to store crc at apecific location of array

const char message[] = "{\"msg\": {\"type\":\"\", \"data\":\"\", \"time\": }, \"crc\": }";
// Packet structure
// <type>-<data>,<time>#<crc>
// Where;
//  -- type: "req" -> pairRequest, "resp" -> pairResponse
//  -- data: contains the actual state of neighbour node
//  data for `type: resp`;    
//       -- "R" -> red, "Y" -> yellow, "G" -> green 
//       -- "0" -> neighbour rejected pair request (because its already pairing with another neighbour)
//  data for `type: req`;
//       -- "pair" -> simple string asking neighbour node accept pair request.
//  -- time: epoch time when this message was created
//  -- crc: checksum of `type` + `data` + `time`


//void sendMessage(uint32_t dest, String msg) {
//  mesh.sendSingle(dest, msg);
//}
//
//String getNodeNameById(uint32_t id) {
//  String strId = String(id);
//  for (uint8_t i = 0; i < sizeof(nodeNameIdRecord) / sizeof(NodeObject); ++i) {
//    if (strId.equals(nodeNameIdRecord[i].id)) {
//        return nodeNameIdRecord[i].name;
//    }
//  }
//  return String("--");
//}
//
//uint32_t getNodeIdByName(String nodeName) {
//  for (uint8_t i = 0; i < sizeof(nodeNameIdRecord) / sizeof(NodeObject); ++i) {
//    if (nodeName.equalsIgnoreCase(nodeNameIdRecord[i].name)) {
//        String strId = nodeNameIdRecord[i].id;
//        uint32_t id = strtoul(strId.c_str(), NULL, 10);
//        return id;
//    }
//  }
//  Serial.println("Unknown node name " + nodeName);
//  return 0;
//}
//
//void ProcessMeshMessage(uint32_t from, String recvMessage){
//  //////////////////////////////////////////////////////////////////
//  //---------------------Decode received message
//  //We receive msg in this form "req-getMin,17263#1261"
//  //Where; 
//  //  --  type = req
//  //  --  data = getMin
//  //  --  time = 17263
//  //  --  crc  = 1261
//  int dashIndex = recvMessage.indexOf('-');
//  int commaIndex = recvMessage.indexOf(','); 
//  int hashIndex = recvMessage.indexOf('#');
//  String recvMsgFrom = getNodeNameById(from);
//  String recvMsgType = recvMessage.substring(0, dashIndex);
//  String recvMsgData = recvMessage.substring(dashIndex + 1, commaIndex);
//  String recvMsgTime = recvMessage.substring(commaIndex + 1, hashIndex);
//  String recvMsgCrc = recvMessage.substring(hashIndex + 1, recvMessage.length()); 
//  //////////////////////////////////////////////////////////////////     
//  
//  DynamicJsonBuffer jsonDynBuf;
//  JsonObject& recvMessageJson = jsonDynBuf.parseObject(message);  
//  recvMessageJson["msg"]["type"] = recvMsgType; 
//  recvMessageJson["msg"]["data"] = recvMsgData;
//  recvMessageJson["msg"]["time"] = recvMsgTime;
//  recvMessageJson["crc"] = recvMsgCrc; 
//
//  Serial.println("RX from  : " + recvMsgFrom);
//  Serial.println("  -- data: " + recvMsgData);
//  Serial.println("  -- type: " + recvMsgType);  
//  Serial.println("  -- crc : " + recvMsgCrc);
//   
//  if (validateRxMessage(recvMessageJson)) {   //check if this msg not already received
//      storeMsgCrc(recvMsgCrc);
//      if (recvMsgType.equalsIgnoreCase("req")) {
//         String respData = "";  
//         if (activeMode == false){         // i am available to get paired
//             respData = currentNodeState;
//         } else if (activeMode == true){   //I am busy (active mode)
//             respData = "0"; 
//         }
//         String msgToSend = createShortMsg("resp", respData); 
//         pout("Send resp to " + recvMsgFrom + ": " + respData);   
//         Serial.println("Sending pair response to origin " + recvMsgFrom + ": " + msgToSend);
//         sendMessage(from, msgToSend);          
//      } else if (recvMsgType.equalsIgnoreCase("resp")) {
//          if (activeMode == true) {           // not timeout (Active mode)
//               if(recvMsgFrom.equalsIgnoreCase(reqNodeName)) {  //check if response is from node to which we sent pair request
//                 
//                 if (!recvMsgData.equals("0")) {
//                     if (currentNodeState.equalsIgnoreCase(recvMsgData)) {
//                       pout("Same state as " + recvMsgFrom + ": " + recvMsgData);
//                     } else {
//                       copyState(recvMsgData);
//                       pout("Copy state of " + recvMsgFrom + ": " + recvMsgData);
//                     }                   
//                 } else if (recvMsgData.equals("0")) {
//                     // pairing node rejected the requst.
//                     // try to pair with another neighbour node
//                     Serial.print("Node " + recvMsgFrom + " is busy... ");
//                     pout("Node " + recvMsgFrom + " is busy...");             
//                 }
//                 activeMode = false;
//                 nextTrigger  = getNextTriggerTime(rate_parameter);
//                 Serial.println("Next poisson " + String(nextTrigger));
//                 nextTriggerCounter = 0;                       
//               } else {
//                 Serial.println("Got response from previous node `" + recvMsgFrom + "`, ignoring!!!");   
//               }
//          } else if (activeMode == true){   // we got ack from last TX node in inActive mode (DISCARD THIS MSG)
//               Serial.println("response received too late, Discarding !");  
//          }
//      } 
//   }
//}
//
//void storeMsgCrc(String crc) {
//    //store the given crc into internal cache memory
//    rcvdMsgCrc[crcStoreIndex] = crc;
//    crcStoreIndex += 1;
//    if (crcStoreIndex > 100) {
//        //flush existing crc's if memory reaches 100
//        crcStoreIndex = 0;
//    }
//}
//
//boolean isDuplicateMsg(String crc){ 
//   //iterate through all stored crc's 
//   for (int i = 0; i < 100; i++) {
//        //check if given crc is already stored in cache memory
//        if (rcvdMsgCrc[i].equals(crc)) {
//            return true;
//        }
//      }  
//    return false;
//}    
//
//boolean validateRxMessage(JsonObject& rxMsg) {
//    String receivedData = rxMsg["msg"]["data"].as<String>();
//    String receivedTime = rxMsg["msg"]["time"].as<String>();      
//    String receivedCrc = rxMsg["crc"].as<String>();
//    String calculatedCrc = calculateCrc(receivedData + "," + receivedTime);
//    if (receivedCrc.equals(calculatedCrc)) {
//        if (isDuplicateMsg(receivedCrc)) {
//            pout("DUP msg " + receivedData + " !!");   
//            Serial.println("duplicate message received. ignoring..."); 
//            return false;
//        } else { 
//           //Serial.println("Message validation successful"); 
//           return true;
//        }
//    } else {
//        Serial.println("CRC did not match.");   
//        return false;
//    }
//}
//
//String calculateCrc(String msgData) {
//    CRC16 crc;
//    crc.processBuffer(msgData.c_str(), strlen(msgData.c_str())); 
//    return String(crc.getCrc()); 
//}
//
//
//// Needed for painless library
//void receivedCallback(uint32_t from, String &msg ) {
//  Serial.printf("Received from %u msg=%s\n", from, msg.c_str());
//  ProcessMeshMessage(from, msg);
//}
//
//void newConnectionCallback(uint32_t nodeId) {
//  //triggered when new node joins network
//  Serial.printf("New Connection, nodeId = %u\n", nodeId);
//}
//
//void changedConnectionCallback() {
//  //triggered when connection change e.g. node exiting the network
//  Serial.printf("Changed connections %s\n", mesh.subConnectionJson().c_str());
//}

void setup() {
  Serial.begin(115200);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
//  SetupMeshNetwork();

  //initializing painlessmesh network drains more current,
  //so better to turn on oLed a few seconds after it finishes
  delay(2000);

//  thisNodeName = getNodeNameById(mesh.getNodeId());
//
//  displayNodeInfo();  // method will populate this nodes children and parent.
//  setNodeState();
//  InitOLed();  
}

//void displayNodeInfo(){
//  Serial.print("Node name: " + thisNodeName);
//  Serial.println(", with following neighbours; ");
//  for (uint8_t i = 0; i < sizeof(allNodesNeighboursList) / sizeof(Neighbours); ++i) {
//     if (thisNodeName.equalsIgnoreCase(allNodesNeighboursList[i].nodeName)) {
//        for (uint8_t j = 0; j < sizeof(allNodesNeighboursList[i].neighbourList)/12; ++j) {
//          if (allNodesNeighboursList[i].neighbourList[j] != "" ) {
//              Serial.print("> ");
//              Serial.println(allNodesNeighboursList[i].neighbourList[j]);   
//              neighbours[totalNeighbours] = allNodesNeighboursList[i].neighbourList[j];
//              totalNeighbours++;         
//          }
//        }
//     }
//  }
//}
//
//void InitOLed() {
//  String initMsg = "Node " + thisNodeName + " (id: " + String(mesh.getNodeId()) + ")";
//  display.init();
//  display.flipScreenVertically();
//  display.setTextAlignment(TEXT_ALIGN_LEFT);
//  display.setFont(ArialMT_Plain_10);
//  display.drawString(0, 0, initMsg);
//  display.drawString(0, line, "State: " + String(currentNodeState));
//  display.display();
//  line += 10;
//}
//
//void SetupMeshNetwork() {
//  //mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); // all types on
//  //mesh.setDebugMsgTypes( ERROR | STARTUP | CONNECTION );  // set before init() so that you can see startup messages
//  mesh.setDebugMsgTypes( ERROR );
//  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT );
//  mesh.onReceive(&receivedCallback);
//  mesh.onNewConnection(&newConnectionCallback);
//  mesh.onChangedConnections(&changedConnectionCallback);
//}
//
//String createShortMsg(String type, String msgData) {
//    //PainlessMesh behaves strange when sending long JSON messages. So we are going to create a short message
//    //by seperating each field with special characters. e.g. 
//    //<type>-<data>,<time>#<crc>
//    //e.g. 
//    //req-getMin,17263#1261
//    String msgTime = String(millis());
//    String calcCrc = calculateCrc(msgData + "," + msgTime);
//
//    //store the msg crc into nodes memory - to avoid receiving this msg again. 
//    storeMsgCrc(calcCrc);
//
//    String msg = type + "-" + msgData + "," + msgTime + "#" + calcCrc; 
//    return msg;
//}
//
//String getRandomNeighbor() {
//  int randomnumber;
//  randomnumber = rand() % totalNeighbours;
//  String randomNodeName = neighbours[randomnumber];
//  return randomNodeName;
//}
//
//void setNodeState() { 
//    int randomIndex = random(sizeof(allPossibleStates)/sizeof(allPossibleStates[0]));
//    Serial.print("random: " + String(randomIndex));
//    currentNodeState = allPossibleStates[randomIndex].nodeState;
//    Serial.println(", State: " + currentNodeState);
//    digitalWrite(allPossibleStates[randomIndex].statePinNumber, HIGH);  
//}
//
//void copyState(String newState){
//  display.setColor(BLACK);
//  display.fillRect(0, 10, 128, 10);
//  display.setColor(WHITE);
//  display.drawString(0, 10, "State: " + newState);
//  currentNodeState = newState;
//  for (int i = 0; i < sizeof(allPossibleStates)/sizeof(allPossibleStates[0]); i++){
//    if (newState.equalsIgnoreCase(allPossibleStates[i].nodeState)) {
//        digitalWrite(allPossibleStates[i].statePinNumber, HIGH);
//    } else {
//        digitalWrite(allPossibleStates[i].statePinNumber, LOW);
//    }
//  }
//}

float getNextTriggerTime(float rate_param){
  int r =  rand();
  float result = -logf(1.0f - (float) r / RAND_MAX) / rate_param;
  Serial.println(String(result));  
  return result;
}

void loop() {
 float addUp = 0; 
 for (int i = 0; i <= 5000; i++){
    nextTrigger = getNextTriggerTime(rate_parameter);
    addUp += nextTrigger;
 }
 float avg = addUp/5000;
 Serial.print("Average: ");
 Serial.println(avg);
 delay(10000);
}

//helper method for oLed
//pout -> Print Out
//void pout(String msg) {
//  if (overwrite == true) {
//    display.setColor(BLACK);
//    display.fillRect(0, line, 128, 64);
//    display.setColor(WHITE);
//    display.drawString(0, line, msg);
//    display.display();
//    line += 10;
//  } else {
//    display.drawString(0, line, msg);
//    display.display();
//    line += 10;
//  }
//
//  if (line == 60) {
//    overwrite = true;
//    line = 20;
//  }
//}
