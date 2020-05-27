 /*
Author  : dhikihandika
Date    : 13/05/2020
*/

//==========================================================================================================================================//
//===================================================|   Initialization Program    |========================================================//                                         
//==========================================================================================================================================//
#include <RTClib.h>                                                 // Add library RTC
#include <Wire.h>                                                   // Add Library Wire RTC communication
#include <SPI.h>                                                    // Add library protocol communication SPI
#include <Ethernet.h>                                               // Add library ethernet
#include <ArduinoJson.h>                                            // Add library arduino json 
#include <PubSubClient.h>                                           // Add library PubSubClient MQTT

// #define DEBUG

#define timer1 5000                                 // timer send command to sensor module 1
#define timer2 10000                                // timer send command to sensor module 2
#define EMG_BUTTON 2                                // define Emergency Button 

#define COM1 32                                     // define LED communication slave1
#define COM2 33                                     // define LED communication slave2 
#define COM3 34

RTC_DS1307 RTC;                                     // Define type RTC as RTC_DS1307 (must be suitable with hardware RTC will be used)

/* configur etheret communication */
byte mac[]  = {0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED };                // MAC Address by see sticker on Arduino Etherent Shield or self determine
IPAddress ip(192, 168, 0, 192);                                    // IP ethernet shield assigned, in one class over the server
IPAddress server(192, 168, 0, 180);                                 // IP LAN (Set ststic IP in PC/Server)
// IPAddress ip(192, 168, 12, 188);                                   // IP ethernet shield assigned, in one class over the server
// IPAddress server(192, 168, 12, 12);                               // IP LAN (Set ststic IP in PC/Server)
int portServer = 1883;                                              // Determine portServer MQTT connection


// Callback function header
void callback(char* topic, byte* payload, unsigned int length);

EthernetClient ethClient;
PubSubClient client(server, 1883, callback, ethClient);


/* variable timer millis */
unsigned long currentMillis = 0;                   
unsigned long previousMillis = 0;

/* global variable to save date and time from RTC */
int year, month, day, hour, minute, second; 
String stringyear, stringmonth, stringday, stringhour, stringminute, stringsecond;

/* variable incoming serverLastData */
uint32_t serverLastData_S1 = 0;
uint32_t serverLastData_S2 = 0;

/* variable check boolean to identify subscribe */
bool timeSubscribe = false;
bool replySubscribe = false;
bool trig_publishFlagRestart = false;
bool Aha_1 = false;
bool Aha_2 = false;

String time;
int flagerror = 0;
int statusTime = 0;
int serverLastMAC01 = 0;
int serverLastMAC02 = 0;
int reply = 0;
int QoS_0 = 0;
int QoS_1 = 1;
int QoS_2 = 2;
int data;

// //==========================================================================================================================================//
// //==========================================================|   Fungsi callback    |========================================================//                                         
// //==========================================================================================================================================//
DynamicJsonBuffer jsonBuffer1;
void callback(char* topic, byte* payload, unsigned int length) {
  #ifdef DEBUG
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  #endif
  
  unsigned char inData[300];
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    inData[(i - 0)] = (const char*)payload[i];
  }

  #ifdef DEBUG
  Serial.println();
  Serial.println("-----------------------");
  Serial.println();
  #endif

  // Parse object JSON from subscribe data timestamp
  JsonObject& root = jsonBuffer1.parseObject(inData); 
  String currentTime = root["current_time"];
  statusTime = root["flagtime"];

  // Parse object JSON from subscribe data reply
  serverLastMAC01 = root["M1"]; 
  serverLastMAC02 = root["M2"]; 
  reply = root["flagreply"]; 
  time = currentTime;

  jsonBuffer1.clear();
} 


//==========================================================================================================================================//
//=========================================================|   Procedure reconnect    |=====================================================//                                         
//==========================================================================================================================================//
void reconnect(){
  while(!client.connected()){
    #ifdef DEBUG
    Serial.print("Attemping MQTT connection...");
    #endif
    if(client.connect("ethernetClient")){
      Serial.println("connected");
      publishFlagStart();
    }else{
      #ifdef DEBUG
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println("try again 5 second");
      #endif
      delay(2000);
    }
  }
}


//==========================================================================================================================================//
//============================================|   Procedure publish data to app server  |===================================================//                                         
//==========================================================================================================================================//
/* publish data sensor 1 */
void publishData_S1(){
  #ifdef DEBUG
  Serial.println("Publish data S1");
  #endif

  RTCprint();                                                                           // Call procedure sync time RTC

/* ArduinoJson create jsonDoc 
Must be know its have a different function 
if you use library ArduinoJson ver 5.x.x or 6.x.x
-- in this program using library ArduinoJson ver 5.x.x
*/
const size_t BUFFER_SIZE = JSON_OBJECT_SIZE(7);                                         // define number of key-value pairs in the object pointed by the JsonObject.

 DynamicJsonBuffer jsonBuffer3(BUFFER_SIZE);                                             // memory management jsonBuffer which is allocated on the heap and grows automatically (dynamic memory)
  JsonObject& JSONencoder = jsonBuffer3.createObject();                                  // createObject function jsonBuffer

  /* Encode object in jsonBuffer */
  JSONencoder["id_controller"] = "CTR01";                                               // key/object its = id_controller
  JSONencoder["id_machine"] = "MAC01_01";                                               // key/object its = id_machine
  JSONencoder["clock"] = stringyear +"-"+stringmonth+"-"+stringday+" "+stringhour+":"+stringminute+":"+stringsecond;
  JSONencoder["count"] = 5;                                                             // key/object its = count
  JSONencoder["status"] = 0;                                                            // key/object its = status
  JSONencoder["temp_data"] = 10000;                                                     // key/object its = temp_data
  JSONencoder["flagsensor"] = 1;                                                        // key/object its = limit

  char JSONmessageBuffer[500];                                                          // array of char JSONmessageBuffer is 500
  JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));                    // “minified” a JSON document

  #ifdef DEBUG
  Serial.println("Sending message to MQTT topic...");                                   // line debugging
  Serial.println(JSONmessageBuffer);                                                    // line debugging
  #endif

  client.publish("PSI/countingbenang/datacollector/reportdata", JSONmessageBuffer);     // publish payload to broker <=> client.publish(topic, payload);

  /* error correction */
  if(client.publish("PSI/countingbenang/datacollector/reportdata", JSONmessageBuffer) == true){
    digitalWrite(COM1, LOW);
    #ifdef DEBUG
    Serial.println("SUCCESS PUBLISHING PAYLOAD S1");
    #endif
  } else {
    #ifdef DEBUG
    Serial.println("ERROR PUBLISHING");
    #endif
  }
}

/* publish data sensor 2 */
void publishData_S2(){
  #ifdef DEBUG
  Serial.println("Publish data S2");
  #endif

  RTCprint();                                                                           // Call procedure sync time RTC

/* ArduinoJson create jsonDoc 
Must be know its have a different function 
if you use library ArduinoJson ver 5.x.x or 6.x.x
-- in this program using library ArduinoJson ver 5.x.x
*/
const size_t BUFFER_SIZE = JSON_OBJECT_SIZE(7);                                         // define number of key-value pairs in the object pointed by the JsonObject.

 DynamicJsonBuffer jsonBuffer3(BUFFER_SIZE);                                             // memory management jsonBuffer which is allocated on the heap and grows automatically (dynamic memory)
  JsonObject& JSONencoder = jsonBuffer3.createObject();                                  // createObject function jsonBuffer

  /* Encode object in jsonBuffer */
  JSONencoder["id_controller"] = "CTR01";                                               // key/object its = id_controller
  JSONencoder["id_machine"] = "MAC02_01";                                            // key/object its = id_machine
  JSONencoder["clock"] = stringyear +"-"+stringmonth+"-"+stringday+" "+stringhour+":"+stringminute+":"+stringsecond;
  JSONencoder["count"] = 8;                                                  // key/object its = count
  JSONencoder["status"] = 0;                                                    // key/object its = status
  JSONencoder["temp_data"] = 20000;                                                   // key/object its = temp_data
  JSONencoder["flagsensor"] = 1;                                                        // key/object its = limit

  char JSONmessageBuffer[500];                                                          // array of char JSONmessageBuffer is 500
  JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));                    // “minified” a JSON document

  #ifdef DEBUG
  Serial.println("Sending message to MQTT topic...");                                   // line debugging
  Serial.println(JSONmessageBuffer);                                                    // line debugging
  #endif

  client.publish("PSI/countingbenang/datacollector/reportdata", JSONmessageBuffer);     // publish payload to broker <=> client.publish(topic, payload);

  /* error correction */
  if(client.publish("PSI/countingbenang/datacollector/reportdata", JSONmessageBuffer) == true){
    digitalWrite(COM2, LOW);
    #ifdef DEBUG
    Serial.println("SUCCESS PUBLISHING PAYLOAD S2");
    #endif
  } else {
    #ifdef DEBUG
    Serial.println("ERROR PUBLISHING");
    #endif
  }
}


//==========================================================================================================================================//
//===================================================|   Procedure publish Flag start  |====================================================//                                         
//==========================================================================================================================================//
void publishFlagStart(){
    if(client.connect("ethernetClient")){
      #ifdef DEBUG
      Serial.println("connected");
      #endif
      // Publish variable startup system
      const size_t restart = JSON_OBJECT_SIZE(2);
      DynamicJsonBuffer jsonBuffer5(restart);
      JsonObject& root = jsonBuffer5.createObject();
      
      root["id_controller"] = "CTR01";
      root["flagstart"] = 1;

      char buffermessage[300];
      root.printTo(buffermessage, sizeof(buffermessage));

      #ifdef DEBUG
      Serial.println("Sending message to MQTT topic...");
      Serial.println(buffermessage);
      #endif

      client.publish("PSI/countingbenang/datacollector/startcontroller", buffermessage);

      if (client.publish("PSI/countingbenang/datacollector/startcontroller", buffermessage) == true){
        #ifdef DEBUG
        Serial.println("Succes sending message");
        Serial.println("--------------------------------------------");
        Serial.println("");
        #endif
      } else {
      #ifdef DEBUG
      Serial.println("ERROR PUBLISHING");
      Serial.println("--------------------------------------------");
      Serial.println("");
      #endif
      }
    } else {
      #ifdef DEBUG
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println("try again 5 second");
      #endif
      delay(2000);
  }
}


//==========================================================================================================================================//
//=======================================================|   Procedure to showData    |=====================================================//                                         
//==========================================================================================================================================//
void showData(){  
  // Setting time must be add size 
  currentMillis = millis();
    if((currentMillis - previousMillis >= 5000)&&(currentMillis - previousMillis < 5100)){
      Aha_1 = true;
      } else {
          if((currentMillis - previousMillis >= 10000)&&(currentMillis - previousMillis < 10100)){
          previousMillis = currentMillis;  
          Aha_2 = true; 
         }
    } 
}
 
 void pub1(){
   if(Aha_1){
    digitalWrite(COM1, HIGH);
    publishFlagStart();
    if(reply == 1){
      Serial.println("Subscribe reply SUCCESS 1");
      Serial.println("");
      publishData_S1();
      reply = 0;
      } else {
        Serial.println("Subscribe reply FAILED 1");
        Serial.println("");
        publishFlagStart();
      } 
   }
  Aha_1 = false;
 }

  void pub2(){
    if(Aha_2){
      digitalWrite(COM2, HIGH);
      publishFlagStart();
      if(reply == 1){
        Serial.println("Subscribe reply SUCCESS 2");
        Serial.println("");
        publishData_S2();
        reply = 0;
        } else {
          Serial.println("Subscribe reply FAILED 2");
          Serial.println("");
          publishFlagStart();
      } 
    }
    Aha_2 = false;
 }


//==========================================================================================================================================//
//================================================|    Procedure lcd Get RealTimeClock   |==================================================//                                         
//==========================================================================================================================================//
void RTCprint(){
  DateTime now = RTC.now();

  #ifdef DEBUG
  Serial.print("date : ");
  Serial.print(now.day(), DEC);
  Serial.print("/");
  Serial.print(now.month(), DEC);
  Serial.print("/");
  Serial.println(now.year(), DEC);
  Serial.print("clock : ");
  Serial.print(now.hour(), DEC);
  Serial.print(":");
  Serial.print(now.minute(), DEC);
  Serial.print(":");
  Serial.print(now.second(), DEC);
  Serial.println(" WIB");
  #endif

  year = now.year(), DEC;
  month = now.month(), DEC;
  day = now.day(), DEC;
  hour = now.hour(), DEC;
  minute = now.minute(), DEC;
  second = now.second(), DEC;
  
  stringyear= String(year);
  stringmonth= String(month);
  stringday= String(day);
  stringhour= String(hour);
  stringminute= String(minute);
  stringsecond= String(second);
}


//==========================================================================================================================================//
//=============================================|   Procedure Sync data and time RTC with Server   |=========================================//                                         
//==========================================================================================================================================//
void syncDataTimeRTC(){
  if(statusTime == 1){
    timeSubscribe = true;
  }

  /* Parse timestamp value */
  year = time.substring(1,5).toInt();
  month = time.substring(6,8).toInt();
  day = time.substring(9,11).toInt();
  hour = time.substring(12,14).toInt();
  minute = time.substring(15,17).toInt();
  second = time.substring(18,20).toInt();

  if(timeSubscribe == true){
    RTC.adjust(DateTime(year, month, day, hour, minute, second));
    timeSubscribe = false;
  }
}


//==========================================================================================================================================//
//================================================|   Procedure Sync lastDatafrom Server   |================================================//                                         
//==========================================================================================================================================//
void syncLastDataServer(){
  if(reply == 1){
  serverLastData_S1 = serverLastMAC01;
  serverLastData_S2 = serverLastMAC02; 
  }
}


//==========================================================================================================================================//
//=========================================================|   Setup Program    |===========================================================//                                         
//==========================================================================================================================================//
void setup(){
    /* Configuration baud rate serial */
    Serial.begin(9600);
    Serial3.begin(9600);

    /* Mode pin definition */
    pinMode(COM1, OUTPUT);
    pinMode(COM2, OUTPUT);
    pinMode(COM3, OUTPUT);
    
    /* Callibration RTC module with NTP Server */
    Wire.begin();
    RTC.begin();
    RTC.adjust(DateTime(__DATE__, __TIME__));       //Adjust data and time from PC every startup
    // RTC.adjust(DateTime(2019, 8, 21, timeClient.getHours(), timeClient.getMinutes(), timeClient.getSeconds()));

    /* Setup Ethernet connection */
    Ethernet.begin(mac, ip);

    /* Setup Broker (server) MQTT Connection */
    client.setServer(server, portServer);
    client.setCallback(callback);
    reconnect();
    
    /* Setup topic subscriber */
    
  if (client.connect("arduinoClient")) {
    client.subscribe("PSI/countingbenang/server/infotimestamp", QoS_0);    //topic get data timestamp from server
    client.subscribe("PSI/countingbenang/server/replystart", QoS_0);    //topic get reset from server
  }
}


//==========================================================================================================================================//
//===========================================================|   Main Loop    |=============================================================//                                         
//==========================================================================================================================================//
void loop(){
  showData();
  pub1();
  pub2();
  syncDataTimeRTC();
  syncLastDataServer();
  client.loop();   // Use to loop callback function
}
