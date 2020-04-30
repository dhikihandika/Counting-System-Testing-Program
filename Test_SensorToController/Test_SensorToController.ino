/*
Author  : dhikihandika
Date    : 24/04/2020
*/

//==========================================================================================================================================//
//===================================================|   Initialization Program    |========================================================//                                         
//==========================================================================================================================================//
#include <RTClib.h>                                                 // Add library RTC
#include <Wire.h>                                                   // Add Library Wire RTC communication
#include <Ethernet.h>                                               // Add library ethernet
#include <SPI.h>                                                    // Add library protocol communication SPI
// #include <ArduinoJson.h>                                            // Add library arduino json 
// #include <PubSubClient.h>                                           // Add library PubSubClient MQTT

#define DEBUG

#define limitData 60000                             // limit countData
#define timer1 5000                                 // timer send command to sensor module 1
#define timer2 10000                                // timer send command to sensor module 2

#define EMG_BUTTON 2                                // define Emergency Button 
#define EMG_LED 34
#define COM1 32                                     // define LED communication slave1
#define COM2 33                                     // define LED communication slave2 


RTC_DS1307 RTC;                                     // Define type RTC as RTC_DS1307 (must be suitable with hardware RTC will be used)

/* configur etheret communication */
byte mac[]  = {0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED };                // MAC Address by see sticker on Arduino Etherent Shield or self determine
IPAddress ip(192, 168, 12, 120);                                     // IP ethernet shield assigned, in one class over the server
IPAddress server(192, 168, 12, 12);                                 // IP LAN (Set ststic IP in PC/Server)
// IPAddress ip(192, 168, 50, 8);                                     // IP ethernet shield assigned, in one class over the server
// IPAddress server(192, 168, 50, 7);                                 // IP LAN (Set ststic IP in PC/Server)
int portServer = 1883;                                              // Determine portServer MQTT connection

// EthernetClient ethClient;                                           // Instance of EthernetClient is a etcClient
// PubSubClient client(ethClient);                                     // Instance of client ia a client

/* variable timer millis */
unsigned long currentMillis = 0;  
unsigned long currentMillis_errorData = 0;     
unsigned long currentMillis_LastValueS1 = 0;    
unsigned long currentMillis_LastValueS2 = 0;                
unsigned long previousMillis = 0;

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

/* variable check boolean to identify subscribe */
bool timeSubscribe = false;
bool replySubscribe = false;
bool trig_publishFlagRestart = false;

int statusReply = 0;


//==========================================================================================================================================//
//=========================================================|   Send Command    |============================================================//                                         
//==========================================================================================================================================//
void sendCommand(){
    currentMillis = millis();
    if(currentMillis - previousMillis == timer1){
        #ifdef DEBUG
        Serial.println("");
        Serial.println("---------------------------------------------------------------");  
        Serial.println("Send command to sensor module 1 (TX3)");
        Serial.println("");
        #endif // DEBUG
        Serial3.print("S_1\n");
    } else {
        if(currentMillis - previousMillis == timer2){
           previousMillis = currentMillis;  
            #ifdef DEBUG
            Serial.println("");
            Serial.println("---------------------------------------------------------------");  
            Serial.println("Send command to sensor module 2 (TX3)");
            Serial.println("");
            #endif // DEBUG
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
      #ifdef DEBUG
      Serial.println("Prefix_A --OK--");
      Serial.print("incomming data= ");Serial.print(incomingData);
      #endif
      
      digitalWrite(COM2, LOW);
      digitalWrite(COM1, HIGH);
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
      diffData_S1 = (data_S1 + serverLastData_S1) - lastData_S1;
      if(diffData_S1<0){
        countData_S1 = diffData_S1 + limitData; 
      } else {
        countData_S1 = diffData_S1;
      }

      //fill lastData
      if((millis() - currentMillis_LastValueS1) > 4000){
        currentMillis_LastValueS1 = millis();
        lastData_S1 = data_S1 + serverLastData_S1;
      }

      RTCprint();

      #ifdef DEBUG
      Serial.print("Current data_S1= ");Serial.print(data_S1); 
      Serial.print(" | diffData_S1= ");Serial.print(diffData_S1); 
      Serial.print(" | status S1= ");Serial.println(status_S1); 
      Serial.println("------------------------------||-------------------------------\n");                                              
      #endif //DEBUG
      } 
  } else {
    /* Show data for sensor 2 */
    if(prefix_B){
      if(stringComplete){
      #ifdef DEBUG
      Serial.println("Prefix_B --OK--");
      Serial.print("incomming data= ");Serial.print(incomingData);
      #endif

      digitalWrite(COM1, LOW);
      digitalWrite(COM2, HIGH);
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

      //Processing Data
      diffData_S2 = (data_S2 + serverLastData_S2) - lastData_S2;
      if(diffData_S2<0){
        countData_S2 = diffData_S2 + limitData; 
      } else {
        countData_S2 = diffData_S2;
      }

      //fill lastDataS2
      if((millis() - currentMillis_LastValueS1) > 4000){
        currentMillis_LastValueS2 = millis();
        lastData_S2 = data_S2 + serverLastData_S2;
      }

      RTCprint();

      #ifdef DEBUG
      Serial.print("Current data_S2= ");Serial.print(data_S2);  
      Serial.print(" | diffData_S2= ");Serial.print(diffData_S2); 
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
    status_S1 = 1;
    errorCheck_S1 = 0;
    Serial.println("=========================");
    Serial.println("        ERROR !!!        ");
    Serial.print("status S1= ");Serial.println(status_S1); 
    Serial.println("=========================");
    Serial.println(" ");
  }
  if(errorCheck_S2 == 3){
    status_S2 = 1;
    errorCheck_S2 = 0;
    Serial.println("=========================");
    Serial.println("        ERROR !!!        ");
    Serial.print("status S2= ");Serial.println(status_S2); 
    Serial.println("=========================");
    Serial.println(" ");
  }
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
    RTC.adjust(DateTime(__DATE__, __TIME__));       //Adjust data and time from PC every startup
    // RTC.adjust(DateTime(2019, 8, 21, timeClient.getHours(), timeClient.getMinutes(), timeClient.getSeconds()));

    /* attachInterrupt Here */
    attachInterrupt(digitalPinToInterrupt(EMG_BUTTON), executeFlagrestart, LOW);
}


//==========================================================================================================================================//
//===========================================================|   Main Loop    |=============================================================//                                         
//==========================================================================================================================================//
void loop(){
    digitalWrite(EMG_LED, LOW);
    sendCommand();
    showData();
    errorData();
    if(trig_publishFlagRestart){
       trig_publishFlagRestart = false;
       Serial.println("Publish flagrestart !!!");
    }
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
      errorCheck_S1 = 0;
    } else {
      if(inChar == 'B'){
      prefix_B = true;
      errorCheck_S2 = 0;
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
    digitalWrite(EMG_LED, HIGH);
    trig_publishFlagRestart = true;
  }
}