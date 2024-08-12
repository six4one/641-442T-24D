#include <Arduino.h>
#include <wire.h> //I2C library
#include <WiFiClientSecure.h>
#include "D:\Personal\Fausto\Documents\PlatformIO\Projects\0_credentials\wifi\home\wifiCredentials.h"
#include "D:\Personal\Fausto\Documents\PlatformIO\Projects\0_credentials\mqtt\linode\mqttCredentials.h"
//#include "D:\Personal\Fausto\Documents\PlatformIO\Projects\0_credentials\certs\linode\serverCert.h"
#include "D:\Personal\Fausto\Documents\PlatformIO\Projects\0_credentials\certs\linode\2024-07-26.h"
#include "D:\Personal\Fausto\Documents\PlatformIO\Projects\0_topics\furnace\topicList-f1.h"
#include <PubSubClient.h>
#include <SPI.h>
#include "Adafruit_MAX31855.h"    //Strain gauge IC Library
//#include "topicList.h"

unsigned long waitCount = 0;                 // counter
uint8_t conn_stat = 0;                       // Connection status for WiFi and MQTT:
                                             //
                                             // status |   WiFi   |    MQTT
                                             // -------+----------+------------
                                             //      0 |   down   |    down
                                             //      1 | starting |    down
                                             //      2 |    up    |    down
                                             //      3 |    up    |  starting
                                             //      4 |    up    | finalising
                                             //      5 |    up    |     up




// MAX31855 digital IO pins mapped to ESP32 module.
#define MAXDO 19
#define MAXCLK 18
#define MAXCS0 17
#define MAXCS1 16

// initialize the Thermocouple #0
Adafruit_MAX31855 thermocouple0(MAXCLK, MAXCS0, MAXDO);

// initialize the Thermocouple #1
Adafruit_MAX31855 thermocouple1(MAXCLK, MAXCS1, MAXDO);

// Pin Map for ESP32 module
int statusLed = 32;
int out0 = 25;
int out1 = 26;
int out2 = 33;
int out3 = 27;
int in0 = 39;
int in1 = 36;
int in2 = 34;
int in3 = 35;

unsigned int length;    //this var represents the length of the string returned from the MQTT callback

String outPayload = "";

boolean tcToggle =0;
boolean in0Previous =0;
boolean in1Previous =0;
boolean in2Previous =0;
boolean in3Previous =0;
boolean in0Current;
boolean in1Current;
boolean in2Current;
boolean in3Current;
boolean in0State = false;
boolean in1State = false;
boolean in2State = false;
boolean in3State = false;

boolean ledState = false;
//boolean txConfirmed = false;
boolean pingConfirmed = false;

unsigned long lastTime =0;
unsigned long lastTimeIn0 =0;
unsigned long lastTimeIn1 =0;
unsigned long lastTimeIn2 =0;
unsigned long lastTimeIn3 =0;
unsigned long currentTime =0;
unsigned long statusFlashTime =0;
unsigned long connectionRetryTime =0;

unsigned long lastStatus = 0;                 // counter in example code for conn_stat == 5
unsigned long lastPing = 0;
unsigned long lastTask = 0;                   // counter in example code for conn_stat <> 5
unsigned long lastLED = 0;                    // counter in example code for led timer
unsigned long pubRate = 0;                    // MQTT initial connection timer

const char* Version = "{\"Version\":\"low_prio_wifi_v2\"}";

int wifiConnectFlashPeriod = 200;
int mqttConnectFlashPeriod = 50;
int connectionRetryPeriod = 10000;

int publishInterval = 5000;   //number of milliseconds for periodic publishing data logging events
int debounceDelay = 20;       //delay to ensure input signal debounce in milliseconds

WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);



void setup() {

  //pinMode(ONBOARD_LED, OUTPUT);
  pinMode(statusLed, OUTPUT);
  pinMode(MAXCS0, OUTPUT);
  pinMode(MAXCS1, OUTPUT);
  pinMode(out0, OUTPUT);
  pinMode(out1, OUTPUT);
  pinMode(out2, OUTPUT);
  pinMode(out3, OUTPUT);
  pinMode(in0, INPUT_PULLDOWN);
  pinMode(in1, INPUT_PULLDOWN);
  pinMode(in2, INPUT_PULLDOWN);
  pinMode(in3, INPUT_PULLDOWN);

  Serial.begin(115200);
  WiFi.mode(WIFI_STA);    // explicitly set mode, esp defaults to STA+AP
 
 /*
  WiFi.begin(wifiSSID, wifiPW);
  espClient.setCACert(six4one_CA);
  //espClient.connect()

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
 
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  
while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
 
    if (client.connect(deviceID, mqttUser, mqttPW )) {
       Serial.println("connected");  
     }else {
        Serial.print("failed with state ");
        Serial.print(client.state());
        delay(2000);
     }
  }
 
  client.publish("esp/test", "Hello from ESP32");
  client.subscribe("esp/test");
*/


//Max31855 Setup
//  Serial.begin(9600);

  while (!Serial) delay(1); // wait for Serial on Leonardo/Zero, etc

  Serial.println("MAX31855 test");
  // wait for MAX chip to stabilize
  delay(500);

  // read the current internal temp of chip 0
  Serial.print("Internal Temp of chip 0 = ");
  digitalWrite(MAXCS0, LOW);
  Serial.println(thermocouple0.readInternal());
  digitalWrite(MAXCS0, HIGH);

  // read the current internal temp of chip 1
  Serial.print("Internal Temp of chip 1 = ");
  digitalWrite(MAXCS1, LOW);
  Serial.println(thermocouple1.readInternal());
  digitalWrite(MAXCS1, HIGH);

  //Status LED test
  digitalWrite(statusLed, HIGH);
  delay(500);
  digitalWrite(statusLed, LOW);

}


void callback(char* inTopic, byte* inPayload, unsigned int length) {
 
  // Note: Do not use the client in the callback to publish, subscribe or
  // unsubscribe as it may cause deadlocks when other things arrive while
  // sending and receiving acknowledgments. Instead, change a global variable,
  // or push to a queue and handle it in the loop after calling `client.loop()`.


  Serial.println("You've got mail!");

  if (strcmp(inTopic,inPing)==0){       //confirmation of conection success from broker
   if (inPayload[0] == '1'){
      //testConfirmed = true;
      pingConfirmed = true;
      digitalWrite(statusLed, LOW); 
      Serial.print(inTopic);
      Serial.println(": TRUE");
    }  
  }

  if (strcmp(inTopic,inTopic0)==0){
   if (inPayload[0] == '1'){
      digitalWrite(out0, HIGH); 
      Serial.print(inTopic);
      Serial.println(": ON");
    }else{
      digitalWrite(out0, LOW);
      Serial.print(inTopic);
      Serial.println(": OFF");
    }  
  }

  if (strcmp(inTopic,inTopic1)==0){
    if (inPayload[0] == '1'){
      digitalWrite(out1, HIGH); 
      Serial.print(inTopic);
      Serial.println(": ON");      
    }else{
      digitalWrite(out1, LOW);
      Serial.print(inTopic);
      Serial.println(": OFF");    
    }
  }

  if (strcmp(inTopic,inTopic2)==0){
    if (inPayload[0] == '1'){
      digitalWrite(out2, HIGH); 
      Serial.print(inTopic);
      Serial.println(": ON");      
    }else{
      digitalWrite(out2, LOW);
      Serial.print(inTopic);
      Serial.println(": OFF");    
    }
  }

  if (strcmp(inTopic,inTopic3)==0){
    if (inPayload[0] == '1'){
      digitalWrite(out3, HIGH); 
      Serial.print(inTopic);
      Serial.println(": ON");      
    }else{
      digitalWrite(out3, LOW);
      Serial.print(inTopic);
      Serial.println(": OFF");    
    }
  }

return;  
}


void loop() {
  currentTime = millis();
// start of non-blocking connection setup section
  if ((WiFi.status() != WL_CONNECTED) && (conn_stat != 1)) {
    conn_stat = 0;
    Serial.println("Connection Status : " + String(conn_stat));
  }
  if ((WiFi.status() == WL_CONNECTED) && !mqttClient.connected() && (conn_stat != 3))  {
    conn_stat = 2;
    Serial.println("Connection Status : " + String(conn_stat));
  }
  if ((WiFi.status() == WL_CONNECTED) && mqttClient.connected() && (conn_stat != 5)) {
    conn_stat = 4;
    Serial.println("Connection Status : " + String(conn_stat));
  }
  switch (conn_stat) {
    case 0:                                                       // MQTT and WiFi down: start WiFi
      Serial.println("MQTT and WiFi down: start WiFi");
      //testConfirmed = false;
      pingConfirmed = false;
      WiFi.disconnect();
      
      WiFi.begin(wifiSSID, wifiPW);
      wifiClient.setCACert(six4one_CA);
      waitCount = 0;
      conn_stat = 1;
      Serial.println("Connection Status : " + String(conn_stat));
      //delay(1000);
      connectionRetryTime = currentTime;
      break;
    case 1:                                                       // WiFi starting, do nothing here
      Serial.println("WiFi starting, wait : "+ String(waitCount));
      waitCount++;
      if (currentTime - statusFlashTime > wifiConnectFlashPeriod){  //Flash status LED to indicate WIFI is connecting
        ledState = !ledState;
        digitalWrite(statusLed, ledState);
        statusFlashTime = currentTime;
      }
      if (currentTime-connectionRetryTime > connectionRetryPeriod){
        Serial.println("Restarting WIFI connection attempt");
        conn_stat = 0;
      }
      //delay(1000);
      break;
    case 2:                                                       // WiFi up, MQTT down: start MQTT
      //MQTTDISCONNECT;
      mqttClient.disconnect();

      Serial.println("WiFi up, MQTT down: start MQTT");
      pingConfirmed = false;
      digitalWrite(statusLed, HIGH);
      mqttClient.setServer(mqttServer, mqttPort);
      mqttClient.setCallback(callback);
      waitCount = 0;
      conn_stat = 3;
      Serial.println("Connection Status : " + String(conn_stat));
      mqttClient.connect(deviceID, mqttUser, mqttPW);
      connectionRetryTime = currentTime;
      //delay(1000);
      break;
    case 3:                                                       // WiFi up, MQTT starting, do nothing here
      Serial.println("Connection Status : " + String(conn_stat));
      Serial.println("WiFi up, MQTT starting, wait : "+ String(waitCount));
      waitCount++;
      if (currentTime - statusFlashTime > 100){  //Flash status LED to indicate MQTT is connecting
        ledState = !ledState;
        digitalWrite(statusLed, ledState);
        statusFlashTime = currentTime;
      }
      if (currentTime-connectionRetryTime > connectionRetryPeriod){
        Serial.println("Restarting MQTT connection attempt");
        waitCount = 0;
        conn_stat = 2;
      }
      //delay(1000);
      break;
    case 4:                                                       // WiFi up, MQTT up: finish MQTT configuration
      
      if (currentTime - pubRate > 5000){
        Serial.println("Connection Status : " + String(conn_stat));
        Serial.println("WiFi up, MQTT up: finish MQTT configuration");
        //digitalWrite(led2,HIGH);
        //lastLED = millis();
        
        //delay(2000);
        //MQTT Subscriptions for control of digital outputs
        mqttClient.subscribe(inPing);
        mqttClient.subscribe(inTopic0);
        mqttClient.subscribe(inTopic1);
        mqttClient.subscribe(inTopic2);
        mqttClient.subscribe(inTopic3);
        
        //Send "Ping" to MQTT broker to test connectivity
        //mqttClient.publish(outPing, "Ping", Version);
        mqttClient.publish(outPing, "Ping");
        Serial.println("Ping sent");
        mqttClient.loop();    //Service the MQTT routine

        pubRate = currentTime;
      }

      if (currentTime - statusFlashTime > mqttConnectFlashPeriod){  //Flash status LED to indicate waiting for "Ping" response from MQTT broker
        ledState = !ledState;
        digitalWrite(statusLed, ledState);
        statusFlashTime = currentTime;
      }

      if (pingConfirmed){
        waitCount = 0;
        digitalWrite(statusLed, HIGH);  
        conn_stat = 5;
        Serial.println("Connection Status : " + String(conn_stat));
      }
      waitCount = 0;
      //delay(1000);                   
      break;
  }
// end of non-blocking connection setup section



// start section with tasks where WiFi/MQTT is required
  if (conn_stat == 5) {
    if (currentTime - lastPing >60000) {       // Send a ping to the broker every 60sec
      pingConfirmed = false;
      Serial.println("Ping");
      mqttClient.publish(outPing, "Ping");     // send status to broker
      mqttClient.loop();                       // give control to MQTT to send message to broker
      lastPing = currentTime;                  // remember time of last sent status message
      digitalWrite(statusLed,LOW);             // pulse satatus LED off to indicate ping sent to broker
    }
    if(currentTime - lastPing > 1000){
      digitalWrite(statusLed,HIGH);            // pulse satatus LED on to indicate ping sent to broker
    }
    //    ArduinoOTA.handle();                      // internal household function for OTA

    //Send periodic logging data to MQTT broker
    if(currentTime-lastTime >= publishInterval){    //Thermocouple data publish period
      if(tcToggle){             //Thermocouple(0) value
        digitalWrite(MAXCS0, LOW);
        double temp = thermocouple0.readCelsius();
        digitalWrite(MAXCS0, HIGH);
        if (isnan(temp)) {
          Serial.println("Something wrong with thermocouple0!");
        } else {
          Serial.print("Temp 0 = ");
          Serial.println(temp);
          Serial.println(" °C");
          //outTopic = "temp"; //temp0Topic;
          outPayload = String(temp);
          if (mqttClient.publish(temp0Topic, (char*) outPayload.c_str())){
            Serial.println ("Publish ok");
            Serial.println(temp0Topic);
            lastTime = currentTime;
          }else {
            Serial.println("Thermocouple0 publish failed");
          }
        }
      }else{             //Thermocouple(1) value
        //Serial.print("Toggle is working");
        digitalWrite(MAXCS1, LOW);
        double temp = thermocouple1.readCelsius();
        digitalWrite(MAXCS1, HIGH);
        if (isnan(temp)) {
          Serial.println("Something wrong with thermocouple1!");
        } else {
          Serial.print("Temp 1 = ");
          Serial.println(temp);
          Serial.println(" °C");
          //outTopic = "temp"; //temp0Topic;
          outPayload = String(temp);
          if (mqttClient.publish(temp1Topic, (char*) outPayload.c_str())){
            Serial.println ("Publish ok");
            Serial.println(temp1Topic);
            lastTime = currentTime;
          }else {
            Serial.println("Thermocouple1 publish failed");
          }
        }      
      }
    tcToggle = !tcToggle;
    }

    // Resolve the hardware inputs and publish their status to MQTT broker
    in0Current = digitalRead(in0);
    if(in0Current != in0Previous){
      if (currentTime-lastTimeIn0 > debounceDelay){;    //debounce delay
        if(in0Current != in0Previous){
          if(in0Current == true){     //rising edge
            outPayload = "1";
            if (mqttClient.publish(outTopic0, (char*) outPayload.c_str())){
              Serial.print(outTopic0);
              Serial.println(": 1");
            }else {
                Serial.print("Publish failed");
            }
          }else{    //falling edge
            outPayload = "0";
            if (mqttClient.publish(outTopic0, (char*) outPayload.c_str())){
              Serial.print(outTopic0);
              Serial.println(": 0");
            }else {
                Serial.print("Publish failed");
            }
          }
          in0Previous = in0Current;   
        }
      }
    }

    in1Current = digitalRead(in1);
    if(in1Current != in1Previous){
      if (currentTime-lastTimeIn1 > debounceDelay){;    //debounce delay
        if(in1Current != in1Previous){
          if(in1Current == true){     //rising edge
            outPayload = "1";
            if (mqttClient.publish(outTopic1, (char*) outPayload.c_str())){
              Serial.print(outTopic1);
              Serial.println(": 1");
            }else {
                Serial.println("Publish failed");
            }
          }else{    //falling edge
            outPayload = "0";
            if (mqttClient.publish(outTopic1, (char*) outPayload.c_str())){
              Serial.print(outTopic1);
              Serial.println(": 0");
            }else {
                Serial.println("Publish failed");
            }
          }
          in1Previous = in1Current;   
        }
      }
    }


    in2Current = digitalRead(in2);
    if(in2Current != in2Previous){
      if (currentTime-lastTimeIn2 > debounceDelay){;    //debounce delay
        if(in2Current != in2Previous){
          if(in2Current == true){     //rising edge
            outPayload = "1";
            if (mqttClient.publish(outTopic2, (char*) outPayload.c_str())){
              Serial.print(outTopic2);
              Serial.println(": 1");
            }else {
                Serial.println("Publish failed");
            }
          }else{    //falling edge
            outPayload = "0";
            if (mqttClient.publish(outTopic2, (char*) outPayload.c_str())){
              Serial.print(outTopic2);
              Serial.println(": 0");
            }else {
                Serial.println("Publish failed");
            }
          }
          in2Previous = in2Current;   
        }
      }
    }

    in3Current = digitalRead(in3);
    if(in3Current != in3Previous){
      if (currentTime-lastTimeIn3 > debounceDelay){;    //debounce delay
        if(in3Current != in3Previous){
          if(in3Current == true){     //rising edge
            outPayload = "1";
            if (mqttClient.publish(outTopic3, (char*) outPayload.c_str())){
              Serial.print(outTopic3);
              Serial.println(": 1");
            }else {
                Serial.println("Publish failed");
            }
          }else{    //falling edge
            outPayload = "0";
            if (mqttClient.publish(outTopic3, (char*) outPayload.c_str())){
              Serial.print(outTopic3);
              Serial.println(": 0");
            }else {
                Serial.println("Publish failed");
            }
          }
          in3Previous = in3Current;   
        }
      }
    }

    mqttClient.loop();    // internal household function for MQTT

  }           // end of section for tasks where WiFi/MQTT are required



// start section for tasks which should run regardless of WiFi/MQTT
currentTime = millis();

  //delay(100);
// end of section for tasks which should run regardless of WiFi/MQTT
}