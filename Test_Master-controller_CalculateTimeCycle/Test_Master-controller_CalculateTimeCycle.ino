/*
Author  : dhikihandika
Date    : 27/05/2020
*/

//==========================================================================================================================================//
//===================================================|   Initialization Program    |========================================================//                                         
//==========================================================================================================================================//
#include <RTClib.h>                                 // Add library RTC
#include <Wire.h>                                   // Add Library Wire RTC communication
#include <Ethernet.h>                               // Add library ethernet
#include <SPI.h>                                    // Add library protocol communication SPI
#include <ArduinoJson.h>                            // Add library arduino json 
#include <PubSubClient.h>                           // Add library PubSubClient MQTT

#define DEBUG
#define DEBUG_TIME

#define limitData 60000                             // limit countData
#define timer1_from 5000                                 // timer send command to sensor module 1
#define timer2_from 10000                                // timer send command to sensor module 2
#define timer1_until 5005                           // timer send command to sensor module 1
#define timer2_until 10005                          // timer send command to sensor module 2

#define EMG_BUTTON 2                                // define Emergency Button

#define COM1 32                                     // define LED communication slave1
#define COM2 33                                     // define LED communication slave2 
#define COM3 34                                     // define LED communication slave3 

RTC_DS1307 RTC;                                     // Define type RTC as RTC_DS1307 (must be suitable with hardware RTC will be used)

/* configur etheret communication */
byte mac[]  = {0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED };                // MAC Address by see sticker on Arduino Etherent Shield or self determine
IPAddress ip(192, 168, 0, 188);                                     // IP ethernet shield assigned, in one class over the server
IPAddress server(192, 168, 0, 180);                                 // IP LAN (Set ststic IP in PC/Server)
// IPAddress ip(192, 168, 12, 188);                                 // IP ethernet shield assigned, in one class over the server
// IPAddress server(192, 168, 12, 12);                              // IP LAN (Set ststic IP in PC/Server)
int portServer = 1883;                                              // Determine portServer MQTT connection

/* Callback function header */
void callback(char* topic, byte* payload, unsigned int length);

EthernetClient ethClient;
PubSubClient client(server, 1883, callback, ethClient);             

/* variable timer millis */
unsigned long currentMillis = 0;  
unsigned long currentMillis_LastValueS1 = 0;          
unsigned long currentMillis_LastValueS2 = 0;
unsigned long currentMillis_errorData = 0;         
unsigned long currentMillis_errorAttemping = 0;
unsigned long previousMillis = 0;
unsigned long previousMillis_errorAttemping = 0;        

/* variable use to be calcuate timecycle program */
unsigned long currentMillis_Reconnect = 0;
unsigned long currentMillis_Cycle1 = 0;
unsigned long currentMillis_Cycle2 = 0;
unsigned long currentMillis_Pub1 = 0;
unsigned long currentMillis_Pub2 = 0;
unsigned long currentMillis_NoGetData1 = 0;
unsigned long currentMillis_NoGetData2 = 0;
unsigned long currentMillis_PrintRTC = 0;
unsigned long currentMillis_SyncRTC = 0;
unsigned long currentMillis_CallbackF = 0;
unsigned long currentMillis_RestartS = 0;


/* global variable to save date and time from RTC */
int year, month, day, hour, minute, second; 
String stringyear, stringmonth, stringday, stringhour, stringminute, stringsecond;

/* variable incoming serverLastData */
uint32_t serverLastData_S1 = 0;
uint32_t serverLastData_S2 = 0;

/* variable last incoming data */
uint32_t lastData_S1 = 0;
uint32_t lastData_S2 = 0;

/* variable incoming data (current data) */
uint32_t data_S1 = 0;
uint32_t data_S2 = 0;

/* variable number initiale with "1" */
uint32_t nuPub = 1; 

/* variable data publish */
uint32_t countData_S1 = 0;
uint32_t countData_S2 = 0;

/* variable status sensor */
int status_S1 = 0;
int status_S2 = 0;

/* varibale indexOf data */
int first = 0;
int last = 0;

/* varibale check status data */
int errorCheck_S1 = 0;
int errorCheck_S2 = 0;

String incomingData = "";                           // a String to hold incoming data
bool stringComplete = false;                        // whether the string is complete

/* varible check boolean prefix of data */
bool prefix_A = false;
bool prefix_B = false;
bool syncLastData_S1 = false;
bool syncLastData_S2 = false;

/* variable check boolean to identify subscribe */
bool trig_publishFlagRestart = false;

String time;
int flagreply = 0;
int statusReply = 0;
int statusTime = 0;
int serverLastMAC01 = 0;
int serverLastMAC02 = 0;
int QoS_0 = 0;
int QoS_1 = 1;
int QoS_2 = 2;
int ledState = LOW;             // ledState used to set the LED


//==========================================================================================================================================//
//==========================================================|   Fungsi callback    |========================================================//                                         
//==========================================================================================================================================//
char data[80];
DynamicJsonBuffer jsonBuffer;
void callback(char* topic, byte* payload, unsigned int length){
  currentMillis_SyncRTC = millis(); currentMillis_CallbackF = millis();
  #ifdef DEBUG
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  #endif

  // Parse data 
  char inData[128];                                 // buffer callback only allow data 128 byte
  for (int i = 0; i < length; i++){
    #ifdef DEBUG
    Serial.print((char)payload[i]);
    #endif
    inData[(i - 0)] = (const char*)payload[i];
  }

  #ifdef DEBUG
  Serial.println();
  Serial.println("-----------------------");
  #endif

  // Parse object JSON from subscribe data timestamp
  JsonObject& root = jsonBuffer.parseObject(inData); 
  String currentTime = root["current_time"];
  statusTime = root["flagtime"];

  // Parse object JSON from subscribe data flagreply
  serverLastMAC01 = root["M1"];
  serverLastMAC02 = root["M2"];
  flagreply = root["flagreply"];
  time = currentTime;
  lastData_S1 = serverLastMAC01; lastData_S2 = serverLastMAC02;
  jsonBuffer.clear();

  unsigned long timeCycle_CallbackF = millis() - currentMillis_CallbackF;
  #ifdef DEBUG
  Serial.print("timeCycle CallbackF: "); Serial.println(timeCycle_CallbackF);
  #endif
} 


//==========================================================================================================================================//
//=========================================================|   Procedure reconnect    |=====================================================//                                         
//==========================================================================================================================================//
void reconnect(){
  currentMillis_Reconnect = millis();
  while(!client.connected()){
    #ifdef DEBUG
    Serial.print("Attemping MQTT connection...");
    #endif
    if(client.connect("arduinoClient")){
      unsigned long timeReconnect = millis() - currentMillis_Reconnect;
      #ifdef DEBUG
      Serial.println("connected");
      Serial.print("timeCycle Reconnect: ");Serial.println(timeReconnect);
      #endif
      // Zero all variable and syc variable time to millis() clock now
      errorCheck_S1 = 0; errorCheck_S2 = 0;
      currentMillis = millis(); previousMillis = millis(); currentMillis_errorData = millis(); 
      // Publish variable startup system
      publishFlagStart();
    }else{
      currentMillis_errorAttemping = millis();
      #ifdef DEBUG
      Serial.print("failed, rc=");  
      Serial.print(client.state());
      Serial.println("try again 5 second");
      #endif
      if(currentMillis_errorAttemping - previousMillis_errorAttemping >= 1000){
        previousMillis_errorAttemping = currentMillis_errorAttemping;
        // if the LED is off turn it on and vice-versa:
        if (ledState == LOW) {
          ledState = HIGH;
          }else{
            ledState = LOW;
          }
        digitalWrite(COM1,ledState);digitalWrite(COM2,ledState);digitalWrite(COM3,ledState);
      }
    }
  }
}

//==========================================================================================================================================//
//===================================================|   Procedure publish Flag start  |====================================================//                                         
//==========================================================================================================================================//
void publishFlagStart(){
    if(client.connect("arduinoClient")){
      #ifdef DEBUG
      Serial.println("connected");
      #endif

      // Publish varcurrentMillis_Sable startup system
      const size_t restart = JSON_OBJECT_SIZE(2);
      DynamicJsonBuffer jsonBuffer(restart);
      JsonObject& root = jsonBuffer.createObject();

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
    }
}


//==========================================================================================================================================//
//==================================================|   Procedure publish Flag Restart  |===================================================//                                         
//==========================================================================================================================================//
/* publish data sensor 1 */
void publishFlagRestart(){
  #ifdef DEBUG
  Serial.println("Publish FlagRestart !!!");
  #endif

/* ArduinoJson create jsonDoc 
Must be know its have a different function 
if you use library ArduinoJson ver 5.x.x or 6.x.x
-- in this program using library ArduinoJson ver 5.x.x
*/
const size_t BUFFER_SIZE = JSON_OBJECT_SIZE(2);                                         // define number of key-value pairs in the object pointed by the JsonObject.

 DynamicJsonBuffer jsonBuffer(BUFFER_SIZE);                                             // memory management jsonBuffer which is allocated on the heap and grows automatically (dynamic memory)
  JsonObject& JSONencoder = jsonBuffer.createObject();                                  // createObject function jsonBuffer

  /* Encode object in jsonBuffer */
  JSONencoder["id_controller"] = "CTR01";                                               // key/object its = id_controller
  JSONencoder["flagrestart"] = 1;                                                       // key/object its = limit

  char JSONmessageBuffer[100];                                                          // array of char JSONmessageBuffer is 100
  JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));                    // “minified” a JSON document

  #ifdef DEBUG
  Serial.println("Sending message to MQTT topic...");                                   // line debugging
  Serial.println(JSONmessageBuffer);                                                    // line debugging
  #endif

  client.publish("PSI/countingbenang/datacollector/startcontroller", JSONmessageBuffer);// publish payload to broker <=> client.publish(topic, payload);

  /* error correction */
  if(client.publish("PSI/countingbenang/datacollector/startcontroller", JSONmessageBuffer) == true){
    unsigned long timeCycle_RestartS = millis() - currentMillis_RestartS;
    #ifdef DEBUG
    Serial.println("SUCCESS PUBLISHING PAYLOAD");
    Serial.print("timeCycle restartS: "); Serial.println(timeCycle_RestartS);
    #endif
    } else {
      #ifdef DEBUG
      Serial.println("ERROR PUBLISHING");
      #endif
    }
}


//==========================================================================================================================================//
//============================================|   Procedure publish data to app server  |===================================================//                                         
//==========================================================================================================================================//
/* publish data sensor 1 */
void publishData_S1(){
  currentMillis_Pub1 = millis();
  #ifdef DEBUG
  Serial.print("Publish data S1= ");
  Serial.println(data_S1);                                                              // line debugging
  #endif

  RTCprint();                                                                           // Call procedure sync time RTC

/* ArduinoJson create jsonDoc 
Must be know its have a different function 
if you use library ArduinoJson ver 5.x.x or 6.x.x
-- in this program using library ArduinoJson ver 5.x.x
*/
const size_t BUFFER_SIZE = JSON_OBJECT_SIZE(8);                                         // define number of key-value pairs in the object pointed by the JsonObject.

 DynamicJsonBuffer jsonBuffer(BUFFER_SIZE);                                             // memory management jsonBuffer which is allocated on the heap and grows automatically (dynamic memory)
  JsonObject& JSONencoder = jsonBuffer.createObject();                                  // createObject function jsonBuffer

  /* Encode object in jsonBuffer */
  JSONencoder["number"] = nuPub; 
  JSONencoder["id_controller"] = "CTR01";                                               // key/object its = id_controller
  JSONencoder["id_machine"] = "MAC01_01";                                               // key/object its = id_machine
  JSONencoder["clock"] = stringyear +"-"+stringmonth+"-"+stringday+" "+stringhour+":"+stringminute+":"+stringsecond;
  JSONencoder["count"] = countData_S1;                                                  // key/object its = count
  JSONencoder["status"] = status_S1;                                                    // key/object its = status
  JSONencoder["temp_data"] = data_S1;                                                   // key/object its = temp_data
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
    if(status_S1 == 0){
      digitalWrite(COM1, LOW);
    }
    nuPub++;
    if(nuPub == 600001){
      nuPub = 0;
    }
    unsigned long timeCycle_Pub1 = millis() - currentMillis_Pub1;
    #ifdef DEBUG
    Serial.println("SUCCESS PUBLISHING PAYLOAD");
    Serial.print("timeCycle publish data 1: "); Serial.println(timeCycle_Pub1);
    #endif
  } else {
    #ifdef DEBUG
    Serial.println("ERROR PUBLISHING");
    #endif
  }
}


/* publish data sensor 2 */
void publishData_S2(){
  currentMillis_Pub2 = millis();
  #ifdef DEBUG
  Serial.print("Publish data S2= ");
  Serial.println(data_S2);                                                              // line debugging
  #endif

  RTCprint();                                                                           // Call procedure sync time RTC

/* ArduinoJson create jsonDoc 
Must be know its have a different function 
if you use library ArduinoJson ver 5.x.x or 6.x.x
-- in this program using library ArduinoJson ver 5.x.x
*/
const size_t BUFFER_SIZE = JSON_OBJECT_SIZE(8);                                         // define number of key-value pairs in the object pointed by the JsonObject.

 DynamicJsonBuffer jsonBuffer(BUFFER_SIZE);                                             // memory management jsonBuffer which is allocated on the heap and grows automatically (dynamic memory)
  JsonObject& JSONencoder = jsonBuffer.createObject();                                  // createObject function jsonBuffer

  /* Encode object in jsonBuffer */
  JSONencoder["number"] = nuPub; 
  JSONencoder["id_controller"] = "CTR01";                                               // key/object its = id_controller
  JSONencoder["id_machine"] = "MAC02_01";                                               // key/object its = id_machine
  JSONencoder["clock"] = stringyear +"-"+stringmonth+"-"+stringday+" "+stringhour+":"+stringminute+":"+stringsecond;
  JSONencoder["count"] = countData_S2;                                                  // key/object its = count
  JSONencoder["status"] = status_S2;                                                    // key/object its = status
  JSONencoder["temp_data"] = data_S2;                                                   // key/object its = temp_data
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
    if(status_S2 == 0){
      digitalWrite(COM2, LOW);
    }
    nuPub++;
    if(nuPub == 60001){
      nuPub = 0;
    }
    unsigned long timeCycle_Pub2 = millis() - currentMillis_Pub2;
    #ifdef DEBUG
    Serial.println("SUCCESS PUBLISHING PAYLOAD");
    Serial.print("timeCycle publish data 1: "); Serial.println(timeCycle_Pub2);
    #endif
  } else {
    #ifdef DEBUG
    Serial.println("ERROR PUBLISHING");
    #endif
  }
}


//==========================================================================================================================================//
//=========================================================|   Send Command    |============================================================//                                         
//==========================================================================================================================================//
void sendCommand(){
    currentMillis = millis();
    if((currentMillis - previousMillis >= timer1_from) && (currentMillis - previousMillis < timer1_until)){
        currentMillis_Cycle1 = millis();
        #ifdef DEBUG
        Serial.println("");
        Serial.println("---------------------------------------------------------------");  
        Serial.println("Send command to sensor module 1 (TX3)");
        Serial.println("");
        #endif // DEBUG
        digitalWrite(COM1, HIGH);                                                  // on LED indicator communication 1
        Serial3.print("S_1\n");
    } else {
        if((currentMillis - previousMillis >= timer2_from) && (currentMillis - previousMillis < timer2_until)){  
            previousMillis = currentMillis;
            currentMillis_Cycle2 = millis();
            #ifdef DEBUG
            Serial.println("");
            Serial.println("---------------------------------------------------------------");  
            Serial.println("Send command to sensor module 2 (TX3)");
            Serial.println("");
            #endif // DEBUG
            digitalWrite(COM2, HIGH);                                                  // on LED indicator communication 2
            Serial3.print("S_2\n");
        } 
    }
}


//==========================================================================================================================================//
//======================================================|   Procedure to showDaota    |=====================================================//                                         
//==========================================================================================================================================//
void showData(){
  /* variable diff data */
  int diffData_S1 = 0;
  int diffData_S2 = 0;

  /* Show data for sensor 1 */
  if(prefix_A){
    if(stringComplete){
      unsigned long timeCycle1 = millis() - currentMillis_Cycle1;
      #ifdef DEBUG
      Serial.print("One timeCycle communication1: "); Serial.println(timeCycle1);
      Serial.println("Prefix_A --OK--");
      Serial.print("incomming data= ");Serial.print(incomingData);
      #endif    
      errorCheck_S1 = 0;
      status_S1 = 0;
      /* remove header and footer */
      first = incomingData.indexOf('A');                                         // determine indexOf 'A'
      last = incomingData.lastIndexOf('/n');                                     // determine lastInndexOf '\n

      /* Parse incoming data to particular variable */ 
      String datasensor1 = incomingData.substring(first, last);                  // substring 
      datasensor1.remove(0,1);                                                   // remove header incomming data
      datasensor1.remove(datasensor1.length()-1, datasensor1.length() - 0);      // remove fotter incomming data (/n)
      data_S1 = datasensor1.toInt();                                             // covert string to integer datasensor1 and save to 'data_S1'

      stringComplete = false;
      prefix_A = false;
      incomingData = "";

      //Processing Data                                       
      diffData_S1 = data_S1 - lastData_S1;
      if(diffData_S1<0){
        countData_S1 = data_S1; 
        } else {
        countData_S1 = diffData_S1;
      }

      // Publish Data
      if(flagreply == 1){
         publishData_S1();
      } else {
         publishFlagStart();
        #ifdef DEBUG
        Serial.println("Not reply anyone data !!!");
        #endif // DEBUG
      } 

      // fill lastDataS2
      if((millis() - currentMillis_LastValueS1) > 4000){
        currentMillis_LastValueS1 = millis();
        lastData_S1 = data_S1;
      }
      
      #ifdef DEBUG
      Serial.print("current data_S1= ");Serial.print(data_S1); 
      Serial.print(" | status S1= ");Serial.println(status_S1); 
      Serial.println("------------------------------||-------------------------------\n");                                              
      #endif //DEBUG
      } 
  } else {
    /* Show data for sensor 2 */
    if(prefix_B){
      if(stringComplete){
      unsigned long timeCycle2 = millis() - currentMillis_Cycle2;
      #ifdef DEBUG
      Serial.print("One timeCycle communication2: "); Serial.println(timeCycle2);
      Serial.println("Prefix_B --OK--");
      Serial.print("incomming data= ");Serial.print(incomingData);
      #endif 

      errorCheck_S2 = 0;
      status_S2 = 0;
      first = incomingData.indexOf('B');                                         // determine indexOf 'A'
      last = incomingData.lastIndexOf('/n');                                     // determine lastInndexOf '\n
      /* When true value is 0 and false is "-1" */

      /* Parse incoming data to particular variable */ 
      String datasensor2 = incomingData.substring(first, last);                  // substring 
      datasensor2.remove(0,1);                                                   // remove header incomming data
      datasensor2.remove(datasensor2.length()-1, datasensor2.length() - 0);      // remove fotter incomming data (/n)
      data_S2 = datasensor2.toInt();                                             // covert string to integer datasensor1 and save to 'data_S1'

      stringComplete = false;
      prefix_B = false;
      incomingData = "";

      // Processing Data
      diffData_S2 = data_S2 - lastData_S2;
      if(diffData_S2<0){
        countData_S2 = data_S2; 
        } else {
        countData_S2 = diffData_S2;
      }
      
      // Publish Data
      if(flagreply == 1){
         publishData_S2();
      } else {
         publishFlagStart();
        #ifdef DEBUG
        Serial.println("Not reply anyone data !!!");
        #endif // DEBUG
      } 

      // fill last data
      if((millis() - currentMillis_LastValueS2) > 4000){
        currentMillis_LastValueS2 = millis();
        lastData_S2 = data_S2;
      }

      #ifdef DEBUG
      Serial.print("current data_S2= ");Serial.print(data_S2); 
      Serial.print(" | status S2= ");Serial.println(status_S2); 
      Serial.println("------------------------------||-------------------------------\n");                                                  
      #endif //DEBUG
      } 
    } 
  }
}


//==========================================================================================================================================//
//==================================================|     Procedure error data        |=====================================================//                                         
//==========================================================================================================================================//
void errorData(){
  if((millis() - currentMillis_errorData)>=5000){
    currentMillis_errorData = millis();
    errorCheck_S1++;
    errorCheck_S2++;
  }

  if(errorCheck_S1 == 3){
    currentMillis_NoGetData1 = millis();
    status_S1 = 1;
    errorCheck_S1 = 0;
    #ifdef DEBUG
    Serial.println("=========================");
    Serial.println("        ERROR !!!        ");
    Serial.print("status S1= ");Serial.println(status_S1);
    #endif 

    // Publish Data
    if(flagreply == 1){
       publishData_S1();
    } else {
        publishFlagStart();
        #ifdef DEBUG
        Serial.println("Not reply anyone data !!!");
        #endif // DEBUG
    } 
    unsigned long timeCycle_NGD1 = millis() - currentMillis_NoGetData1;
    #ifdef DEBUG
    Serial.println("=========================");
    Serial.println(" ");
    Serial.print("timeCycle NGD1: "); Serial.println(timeCycle_NGD1);
    #endif
  }

  if(errorCheck_S2 == 3){
    currentMillis_NoGetData2 = millis();
    status_S2 = 1;
    errorCheck_S2 = 0;
    #ifdef DEBUG
    Serial.println("=========================");
    Serial.println("        ERROR !!!        ");
    Serial.print("status S2= ");Serial.println(status_S2); 
    #endif
    // Publish Data
    if(flagreply == 1){
       publishData_S2();
    } else {
        publishFlagStart();
        #ifdef DEBUG
        Serial.println("Not reply anyone data !!!");
        #endif // DEBUG
    } 
    unsigned long timeCycle_NGD2 = millis() - currentMillis_NoGetData2;
    #ifdef DEBUG
    Serial.println("=========================");
    Serial.println(" ");
    Serial.print("timeCycle NGD2: "); Serial.println(timeCycle_NGD2);
    #endif
  }
}


//==========================================================================================================================================//
//================================================|    Procedure lcd Get RealTimeClock   |==================================================//                                         
//==========================================================================================================================================//
void RTCprint(){
  currentMillis_PrintRTC = millis();
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

  unsigned long timeCycle_printRTC = millis() - currentMillis_PrintRTC;
  #ifdef DEBUG
  Serial.print("Time Cycle print RTC: "); Serial.println(timeCycle_printRTC);
  #endif
}


//==========================================================================================================================================//
//=============================================|   Procedure Sync data and time RTC with Server   |=========================================//                                         
//==========================================================================================================================================//
void syncDataTimeRTC(){
  if(statusTime == 1){
    status_S1 = 0;
    status_S2 = 0;

    /* Parse timestamp value */
    year = time.substring(1,5).toInt();
    month = time.substring(6,8).toInt();
    day = time.substring(9,11).toInt();
    hour = time.substring(12,14).toInt();
    minute = time.substring(15,17).toInt();
    second = time.substring(18,20).toInt();

    RTC.adjust(DateTime(year, month, day, hour, minute, second));
    statusTime = 0;

    unsigned long timeCycle_SyncRTC = millis() - currentMillis_SyncRTC;
    #ifdef DEBUG
    Serial.print("Timc Cycle SyncRTC: "); Serial.println(timeCycle_SyncRTC);
    #endif
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
    pinMode(EMG_BUTTON, INPUT_PULLUP);
    
    /* Callibration RTC module with NTP Server */
    Wire.begin();
    RTC.begin();
    RTC.adjust(DateTime(__DATE__, __TIME__));       //Adjust data and time from PC currentMillis_Svery startup
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

    /* reserve 200 bytes for the incomingData*/
    incomingData.reserve(200);

    /* attachInterrupt Here */
    attachInterrupt(digitalPinToInterrupt(EMG_BUTTON), executeFlagrestart, LOW);

    /* debug timecycle */
}


//==========================================================================================================================================//
//===========================================================|   Main Loop    |=============================================================//                                         
//==========================================================================================================================================//
void loop(){
    syncDataTimeRTC();
    reconnect();
    sendCommand();
    showData();
    errorData();
    if(trig_publishFlagRestart){
      trig_publishFlagRestart = false;
      publishFlagRestart();
    }
    client.loop();   // Use to loop callback function
}


//==========================================================================================================================================//
//==========================================================|   Serial ISR    |=============================================================//                                         
//==========================================================================================================================================//
/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent3(){
  while (Serial3.available()){
    // get the new byte:
    char inChar = (char)Serial3.read();
    // add it to the incomingData
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    incomingData += inChar;
    if(inChar == 'A'){
      prefix_A = true;
    } else {
      if(inChar == 'B'){
      prefix_B = true;
      } else {
        if(inChar == '\n'){ 
          stringComplete = true;
        }
      }
    }
  }
}


//==========================================================================================================================================//
//=========================================================|   Digital ISR    |=============================================================//                                         
//==========================================================================================================================================//
void executeFlagrestart(){
  if(digitalRead(EMG_BUTTON) == LOW){
    trig_publishFlagRestart = true;
    currentMillis_RestartS = millis();
  }
}
