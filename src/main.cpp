//************************************************************
// this is the sketch running on loggia_front node
//
// 1. 
// 2. 
//
//
//************************************************************
#include <Arduino.h>
#include "painlessMesh.h"
#include <TaskScheduler.h>
#include "node_list.h"
#include "config.h"
// include library to read and write from flash memory
#include <EEPROM_info.h>
//For queue related to PIR
#include <cppQueue.h>
#include "esp8266_mutex.h"

#define   MESH_PREFIX     "whatever"
#define   MESH_PASSWORD   "somethingSneaky"
#define   MESH_PORT       5555

Scheduler userScheduler; // to control your personal task
painlessMesh  mesh;

// User stub
unsigned int HUMIDITY_TEMPERATURE_SAMPLING_PERIOD = 60;
void sendMeasurements() ; // Prototype so PlatformIO doesn't complain
Task taskSendMeasurements( TASK_SECOND * HUMIDITY_TEMPERATURE_SAMPLING_PERIOD , TASK_FOREVER, &sendMeasurements );

// User stub
float CHECK_PIR_SAMPLING_PERIOD = 0.5;
void checkPIR() ; // Prototype so PlatformIO doesn't complain
Task taskCheckPIR( TASK_SECOND * CHECK_PIR_SAMPLING_PERIOD , TASK_FOREVER, &checkPIR );

mutex_t mutex_PIR_queue;

cppQueue  PIR_queue(sizeof(uint8_t), 100, FIFO);  // Instantiate queue

bool PIR_GATE_DETECTED = false, PIR_NEIGHBORHOOD_DETECTED = false, PIR_DOOR_DETECTED = false, PIR_SWIMMINGPOOL_DETECTED = false;
void checkPIR(){
  uint8_t swimmingpool_pir = SWIMMINGPOOL_PIR, gate_pir = GATE_PIR, neighborhood_pir = NEIGHBORHOOD_PIR, door_pir = DOOR_PIR;
  unsigned int voltValuePIR_SWIMMINGPOOL_act = analogRead(SWIMMINGPOOL_PIR_PIN);
  unsigned int voltValuePIR_GATE_act = analogRead(GATE_PIR_PIN);
  unsigned int voltValuePIR_DOOR_act = analogRead(DOOR_PIR_PIN);
  unsigned int voltValuePIR_NEIGHBORHOOD_act = analogRead(NEIGHBORHOOD_PIR_PIN);
  
  Serial.printf("%d\n",voltValuePIR_SWIMMINGPOOL_act);

  if(voltValuePIR_SWIMMINGPOOL_act >= DETECTION_THRESHOLD){
    if(PIR_SWIMMINGPOOL_DETECTED == false){
      GetMutex(&mutex_PIR_queue);
      for(unsigned int i= 0; i< MESSAGE_PER_PIR_DETECTED; i++)
        PIR_queue.push(&(swimmingpool_pir));
      ReleaseMutex(&mutex_PIR_queue);
      PIR_SWIMMINGPOOL_DETECTED = true;
    }
  } else{
    PIR_SWIMMINGPOOL_DETECTED = false;
  }

  if(voltValuePIR_NEIGHBORHOOD_act >= DETECTION_THRESHOLD){
    if(PIR_NEIGHBORHOOD_DETECTED == false){
      GetMutex(&mutex_PIR_queue);
      for(unsigned int i= 0; i< MESSAGE_PER_PIR_DETECTED; i++)
        PIR_queue.push(&(neighborhood_pir));
      ReleaseMutex(&mutex_PIR_queue);
      PIR_NEIGHBORHOOD_DETECTED = true;
    }
  } else{
    PIR_NEIGHBORHOOD_DETECTED = false;
  }

  if(voltValuePIR_DOOR_act >= DETECTION_THRESHOLD){
    if(PIR_DOOR_DETECTED == false){
      GetMutex(&mutex_PIR_queue);
      for(unsigned int i= 0; i< MESSAGE_PER_PIR_DETECTED; i++)
        PIR_queue.push(&(door_pir));
      ReleaseMutex(&mutex_PIR_queue);
      PIR_DOOR_DETECTED = true;
    }
  } else{
    PIR_DOOR_DETECTED = false;
  }

  if(voltValuePIR_GATE_act >= DETECTION_THRESHOLD){
    if(PIR_GATE_DETECTED == false){
      GetMutex(&mutex_PIR_queue);
      for(unsigned int i= 0; i< MESSAGE_PER_PIR_DETECTED; i++)
        PIR_queue.push(&(gate_pir));
      ReleaseMutex(&mutex_PIR_queue);
      PIR_GATE_DETECTED = true;
    }
  } else{
    PIR_GATE_DETECTED = false;
  }
}

//unsigned int DURATION_ON_GATE_LIGHT = 10;
/*void switchOffGateLights() ; // Prototype so PlatformIO doesn't complain
Task taskSwitchOffGateLights( TASK_SECOND * DURATION_ON_GATE_LIGHT , TASK_ONCE, &switchOffGateLights );*/

//unsigned int DURATION_ON_NEIGHBORHOOD_LIGHT = 10;
/*void switchOffNeighborhoodLights() ; // Prototype so PlatformIO doesn't complain
Task taskSwitchOffNeighborhoodLights( TASK_SECOND * DURATION_ON_NEIGHBORHOOD_LIGHT , TASK_ONCE, &switchOffNeighborhoodLights );*/
//
/*bool forced_status_gate_light = false;//Setting to true to avoid PIR can switch it off
bool forced_status_neighborhood_light = false;//Setting to true to avoid PIR can switch it off

bool periodical_gate_light = false; //Setting to true if lights are ON due to periodical scheduling
bool periodical_neighborhood_light = false;*/
//

/*void switchOffGateLights() {
  digitalWrite(LED_BUILTIN, HIGH);
  //Switch off lights  
  digitalWrite(GATE_LIGHT_PIN,HIGH);

  taskSwitchOffGateLights.disable();
}*/

/*void switchOffNeighborhoodLights() {
  digitalWrite(LED_BUILTIN, HIGH);
  //Switch off lights  
  digitalWrite(NEIGHBORHOOD_LIGHT_PIN,HIGH);

  taskSwitchOffNeighborhoodLights.disable();
}*/
void sendMex(unsigned int type,String response, String value, uint32_t dest);

void sendMeasurements() {
  
  String mex;

  StaticJsonDocument<200> doc;

  doc["response"] = "humidity_temperature";

  JsonObject value = doc.createNestedObject("value");

  //Read from DHT22
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  //int temperature = rand()%50;
  //int humidity = rand()%50;
  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);

  value["temperature"] = t;
  value["humidity"] = h;

  serializeJson(doc, mex);

  mesh.sendSingle(uint32_t(MQTTBRIDGE),mex);

  //String msg = "Hello from node ";
  //msg += mesh.getNodeId();
  
  //taskSendMessage.setInterval( random( TASK_SECOND * 1, TASK_SECOND * 5 ));
}

void sendMex(unsigned int type,String type_payload, String value, uint32_t dest){

  String mex;

  StaticJsonDocument<200> doc;

  if(type == RESPONSE)
    doc["response"] = type_payload;
  else if(type == REQUEST)
    doc["request"] = type_payload;
  else 
    doc["error"] = type_payload;

  doc["value"] = value;

  serializeJson(doc, mex);

  mesh.sendSingle(dest,mex);

}

// ********************************************************
// Needed for painless library
// messages received from painless mesh network
void receivedCallback( const uint32_t &from, const String &msg ) 
  {
    Serial.printf("tile: Received from %u msg=%s\n", from, msg.c_str());

    StaticJsonDocument<200> jsonBuffer;

    // Deserialize the JSON document
    DeserializationError error = deserializeJson(jsonBuffer, msg);

    // Test if parsing succeeds.
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.c_str());
      return;
    }
    
    if ( (jsonBuffer.containsKey("request")) && (jsonBuffer.containsKey("value")) ) {
      //Message destinated to MQTT
      String request = jsonBuffer["request"];
      String value = jsonBuffer["value"];

      /*if(request == "gate_light_status"){
        
        if(value == "OFF"){
          digitalWrite(LED_BUILTIN, HIGH);

          //Switch off lights  
          digitalWrite(GATE_LIGHT_PIN,HIGH);
          forced_status_gate_light = false; // to reset
        }
        if(value == "ON"){
          digitalWrite(LED_BUILTIN, LOW);

          //Switch on lights
          digitalWrite(GATE_LIGHT_PIN,LOW);
          forced_status_gate_light = true; //To avoid PIR can switch it off
        }

        Serial.printf("target: %u\n",MQTTBRIDGE);
        sendMex(RESPONSE,"gate_light_status",value,MQTTBRIDGE);
        
      }else if(request == "neighborhood_light_status"){
        
        if(value == "OFF"){
          digitalWrite(LED_BUILTIN, HIGH);

          //Switch off lights     
          digitalWrite(NEIGHBORHOOD_LIGHT_PIN,HIGH);
          forced_status_neighborhood_light = false; //To reset
        }
        if(value == "ON"){
          digitalWrite(LED_BUILTIN, LOW);

          //Switch on lights
          digitalWrite(NEIGHBORHOOD_LIGHT_PIN,LOW);
          forced_status_neighborhood_light = true; //To avoid PIR can switch it off
        }

        sendMex(RESPONSE,"neighborhood_light_status",value,MQTTBRIDGE);
        
      }else if(request == "gate_light_status_PIR"){
        //Switch on lights and start countdown if light has not been switched ON by GUI or by periodical scheduling
        if(forced_status_gate_light != true && periodical_gate_light != true){

          digitalWrite(LED_BUILTIN, LOW);

          //Switch on lights
          digitalWrite(GATE_LIGHT_PIN,LOW);

          //Enable task to switch it off
          if(taskSwitchOffGateLights.isEnabled())
            taskSwitchOffGateLights.restart();
          else
            taskSwitchOffGateLights.enable();

        }

      }else if(request == "neighborhood_light_status_PIR"){
        //Switch on lights and start countdown if light has not been switched ON by GUI or by periodical scheduling
        if(forced_status_neighborhood_light != true && periodical_neighborhood_light != true){

          digitalWrite(LED_BUILTIN, LOW);

          //Switch on lights
          digitalWrite(NEIGHBORHOOD_LIGHT_PIN,LOW);

          //Enable task to switch it off or reset it in case it has been already enabled
          if(taskSwitchOffNeighborhoodLights.isEnabled())
            taskSwitchOffNeighborhoodLights.restart();
          else
            taskSwitchOffNeighborhoodLights.enable();

        }

      }else if(request == "gate_light_duration"){
        DURATION_ON_GATE_LIGHT = (unsigned int)value.toInt();

        sendMex(RESPONSE,"gate_light_duration",value,MQTTBRIDGE);

        EEPROM.put(DURATION_ON_GATE_LIGHT_FLASH_ADDRESS,DURATION_ON_GATE_LIGHT);
        EEPROM.commit();

      }else if(request == "neighborhood_light_duration"){
        DURATION_ON_NEIGHBORHOOD_LIGHT = (unsigned int)value.toInt();

        sendMex(RESPONSE,"neighborhood_light_duration",value,MQTTBRIDGE);

        EEPROM.put(DURATION_ON_NEIGHBORHOOD_LIGHT_FLASH_ADDRESS,DURATION_ON_NEIGHBORHOOD_LIGHT);
        EEPROM.commit();

      }else */if(request == "humidity_temperature_sampling_period"){
        //Set the new interval
        unsigned int new_sampling_period = (unsigned int)value.toInt();
        HUMIDITY_TEMPERATURE_SAMPLING_PERIOD = new_sampling_period;
        taskSendMeasurements.setInterval(HUMIDITY_TEMPERATURE_SAMPLING_PERIOD);

        sendMex(RESPONSE,"humidity_temperature_sampling_period",value,MQTTBRIDGE);

        EEPROM.put(HUMIDITY_TEMPERATURE_SAMPLING_PERIOD_FLASH_ADDRESS,HUMIDITY_TEMPERATURE_SAMPLING_PERIOD);
        EEPROM.commit();

      }else if(request == "new_root"){
        
        ROOT = (unsigned int)value.toInt();

        EEPROM.put(ROOT_NODE_FLASH_ADDRESS,ROOT);
        EEPROM.commit();

      }else if(request == "new_MQTT_bridge"){
        
        MQTTBRIDGE = (unsigned int)value.toInt();

        EEPROM.put(MQTTBRIDGE_NODE_FLASH_ADDRESS,MQTTBRIDGE);
        EEPROM.commit();

      }else if(request == "new_gate"){
        
        GATE = (unsigned int)value.toInt();

        EEPROM.put(GATE_NODE_FLASH_ADDRESS,GATE);
        EEPROM.commit();

      }else if(request == "new_neighborhood"){
        
        NEIGHBORHOOD = (unsigned int)value.toInt();

        EEPROM.put(NEIGHBORHOOD_NODE_FLASH_ADDRESS,NEIGHBORHOOD);
        EEPROM.commit();

      }else if(request == "new_swimmingpool"){
        
        SWIMMINGPOOL = (unsigned int)value.toInt();

        EEPROM.put(SWIMMINGPOOL_NODE_FLASH_ADDRESS,SWIMMINGPOOL);
        EEPROM.commit();

      }else if(request.endsWith("reboot")){
        
        ESP.restart();

      }else{

        printf("Unknown request!\n");

      }
      
    }else{

        printf("Unknown format of message!\n");

      }
  
  }

void newConnectionCallback(uint32_t nodeId) {
    Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);

    Serial.print("List");
    Serial.print(mesh.subConnectionJson());
}

void changedConnectionCallback() {
  Serial.printf("Changed connections\n");

  auto nodes = mesh.getNodeList(true);
  bool swimmingpool_node_found = false;
  bool gate_node_found = false;
  for (auto &&id : nodes){
    if(id == SWIMMINGPOOL)
      swimmingpool_node_found = true;
    if(id == GATE)
      gate_node_found = true;
  }

  if(swimmingpool_node_found)
    digitalWrite(SWIMMINGPOOL_SWITCH_NODE_DETECTED_LED,HIGH);
  else
    digitalWrite(SWIMMINGPOOL_SWITCH_NODE_DETECTED_LED,LOW);

  if(gate_node_found)
    digitalWrite(GATE_SWITCH_NODE_DETECTED_LED,HIGH);
  else
    digitalWrite(GATE_SWITCH_NODE_DETECTED_LED,LOW);  
}

void nodeTimeAdjustedCallback(int32_t offset) {
    Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(),offset);
}

//***************************************************************************

void retrieve_EEPROM_values(){
  EEPROM.get(HUMIDITY_TEMPERATURE_SAMPLING_PERIOD_FLASH_ADDRESS,HUMIDITY_TEMPERATURE_SAMPLING_PERIOD);
  EEPROM.get(ROOT_NODE_FLASH_ADDRESS,ROOT);
  EEPROM.get(MQTTBRIDGE_NODE_FLASH_ADDRESS,MQTTBRIDGE);
  EEPROM.get(GATE_NODE_FLASH_ADDRESS,GATE);
  EEPROM.get(SWIMMINGPOOL_NODE_FLASH_ADDRESS,SWIMMINGPOOL);
  EEPROM.get(NEIGHBORHOOD_NODE_FLASH_ADDRESS,NEIGHBORHOOD);

  Serial.printf("HUMIDITY_TEMPERATURE_SAMPLING_PERIOD %u\n",HUMIDITY_TEMPERATURE_SAMPLING_PERIOD);
  Serial.printf("ROOT %u\n",ROOT);
  Serial.printf("MQTTBRIDGE %u\n",MQTTBRIDGE);
  Serial.printf("GATE %u\n",GATE);
  Serial.printf("SWIMMINGPOOL %u\n",SWIMMINGPOOL);
  Serial.printf("NEIGHBORHOOD %u\n",NEIGHBORHOOD);
}

//Call once at the very beginning to initialize EEPROM
void store_EEPROM_values(){
  EEPROM.put(HUMIDITY_TEMPERATURE_SAMPLING_PERIOD_FLASH_ADDRESS,HUMIDITY_TEMPERATURE_SAMPLING_PERIOD);
  EEPROM.put(ROOT_NODE_FLASH_ADDRESS,ROOT);
  EEPROM.put(MQTTBRIDGE_NODE_FLASH_ADDRESS,MQTTBRIDGE);
  EEPROM.put(GATE_NODE_FLASH_ADDRESS,GATE);
  EEPROM.put(SWIMMINGPOOL_NODE_FLASH_ADDRESS,SWIMMINGPOOL);
  EEPROM.put(NEIGHBORHOOD_NODE_FLASH_ADDRESS,NEIGHBORHOOD);

  EEPROM.commit();

  /*
  Serial.printf("HUMIDITY_TEMPERATURE_SAMPLING_PERIOD %u\n",HUMIDITY_TEMPERATURE_SAMPLING_PERIOD);
  Serial.printf("ROOT %u\n",ROOT);
  Serial.printf("MQTTBRIDGE %u\n",MQTTBRIDGE);
  Serial.printf("GATE %u\n",GATE);
  Serial.printf("SWIMMINGPOOL %u\n",SWIMMINGPOOL);
  Serial.printf("NEIGHBORHOOD %u\n",NEIGHBORHOOD);
  */
}
/*
//Interrupt service routine
//Check how many time the ISR is called..
ICACHE_RAM_ATTR void GATE_movement_detection() {
  //Serial.println("GATE MOTION DETECTED!!!");
  uint8_t gate_pir = GATE_PIR;
  PIR_queue.push(&(gate_pir));
  //digitalWrite(led, HIGH);
  //startTimer = true;
  //lastTrigger = millis();
}

ICACHE_RAM_ATTR void SWIMMINGPOOL_movement_detection() {
  //Serial.println("SWIMMINGPOOL MOTION DETECTED!!!");
  uint8_t swimmingpool_pir = SWIMMINGPOOL_PIR;
  PIR_queue.push(&(swimmingpool_pir));
}

ICACHE_RAM_ATTR void DOOR_movement_detection() {
  //Serial.println("DOOR MOTION DETECTED!!!");
  uint8_t door_pir = DOOR_PIR;
  PIR_queue.push(&(door_pir));
}

ICACHE_RAM_ATTR void NEIGHBORHOOD_movement_detection() {
  //Serial.println("NEIGHBORHOOD MOTION DETECTED!!!");
  uint8_t neighborhood_pir = NEIGHBORHOOD_PIR;
  PIR_queue.push(&(neighborhood_pir));
}
*/

void setup() {
  Serial.begin(115200);

//mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); // all types on
  mesh.setDebugMsgTypes( ERROR | STARTUP );  // set before init() so that you can see startup messages

  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT, WIFI_AP_STA, 1, 0, MAX_CONNECTION_PER_NODE );
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

  // if you want your node to accept OTA firmware, simply include this line
  // with whatever role you want your hardware to be. For instance, a
  // mesh network may have a thermometer, rain detector, and bridge. Each of
  // those may require different firmware, so different roles are preferrable.
  //
  // MAKE SURE YOUR UPLOADED OTA FIRMWARE INCLUDES OTA SUPPORT OR YOU WILL LOSE
  // THE ABILITY TO UPLOAD MORE FIRMWARE OVER OTA. YOU ALSO WANT TO MAKE SURE
  // THE ROLES ARE CORRECT
  mesh.initOTAReceive("lf"); //loggia_front


  // initialize EEPROM with predefined size
  EEPROM.begin(EEPROM_SIZE);
  //store_EEPROM_values();
  retrieve_EEPROM_values();
  Serial.printf("target: %u\n",MQTTBRIDGE);

  userScheduler.addTask( taskSendMeasurements );
  userScheduler.addTask( taskCheckPIR );
/*  userScheduler.addTask( taskSwitchOffGateLights ); //For PIR
  userScheduler.addTask( taskSwitchOffNeighborhoodLights ); //For PIR
*/
  taskSendMeasurements.enable();
  taskCheckPIR.enable();

  pinMode(LED_BUILTIN, OUTPUT);
  
  // PIR Motion Sensor mode INPUT_PULLUP so that when open circuit -> V = VCC
  pinMode(GATE_PIR_PIN, INPUT);
  pinMode(DOOR_PIR_PIN, INPUT);
  pinMode(SWIMMINGPOOL_PIR_PIN, INPUT);
  pinMode(NEIGHBORHOOD_PIR_PIN, INPUT);
  // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode (or FALLING).. check with PIR SENSOR
  /*attachInterrupt(digitalPinToInterrupt(GATE_PIR_PIN), GATE_movement_detection, FALLING);
  attachInterrupt(digitalPinToInterrupt(DOOR_PIR_PIN), DOOR_movement_detection, FALLING);
  attachInterrupt(digitalPinToInterrupt(SWIMMINGPOOL_PIR_PIN), SWIMMINGPOOL_movement_detection, FALLING);
  attachInterrupt(digitalPinToInterrupt(NEIGHBORHOOD_PIR_PIN), NEIGHBORHOOD_movement_detection, FALLING);*/

  //mesh.setRoot(true);

  // This node and all other nodes should ideally know the mesh contains a root, so call this on all nodes
  mesh.setContainsRoot(true);

  //Used to check if the related smart switch has been detected
  pinMode(SWIMMINGPOOL_SWITCH_NODE_DETECTED_LED, OUTPUT);
  pinMode(GATE_SWITCH_NODE_DETECTED_LED, OUTPUT);
  digitalWrite(GATE_SWITCH_NODE_DETECTED_LED, LOW);
  digitalWrite(SWIMMINGPOOL_SWITCH_NODE_DETECTED_LED, LOW);

  Serial.printf("ID %u\n",mesh.getNodeId());

  //Initialize the DHT sensor
  dht.begin();

}

void loop() {
  // it will run the user scheduler as well
  mesh.update();

  if(!PIR_queue.isEmpty()){
    uint8_t PIR_event;
    GetMutex(&mutex_PIR_queue);
    PIR_queue.pop(&PIR_event);
    ReleaseMutex(&mutex_PIR_queue);

    switch (PIR_event)
    {
    case GATE_PIR:
      Serial.println("GATE MOTION DETECTED!!!");
      sendMex(REQUEST,"gate_light_PIR","1",GATE);
      break;

    case DOOR_PIR:
      Serial.println("DOOR MOTION DETECTED!!!");
      sendMex(REQUEST,"gate_light_PIR","1",GATE);
      break;

    case SWIMMINGPOOL_PIR:
      Serial.println("SWIMMINGPOOL MOTION DETECTED!!!");
      sendMex(REQUEST,"swimmingpool_light_PIR","1",SWIMMINGPOOL);
      break;

    case NEIGHBORHOOD_PIR:
      Serial.println("NEIGHBORHOOD MOTION DETECTED!!!");
      sendMex(REQUEST,"neighborhood_light_PIR","1",NEIGHBORHOOD);
      break;
    
    default:
      break;
    }
  }
}