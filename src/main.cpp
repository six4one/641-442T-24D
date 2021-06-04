#include <Arduino.h>
#include <wire.h>
#include <WiFiClientSecure.h>
#include "D:\Personal\Fausto\Documents\PlatformIO\Projects\Linode Network Credentials\credentials.h"
#include <PubSubClient.h>
#include "D:\Personal\Fausto\Documents\PlatformIO\Projects\Linode Network Credentials\serverCert.h"
#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include "topicList.h"

WiFiClientSecure espClient;
PubSubClient client(espClient);

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

unsigned int length;

String outPayload = "";
//String outTopic = "";

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

unsigned long lastTime =0;
unsigned long lastTimeIn0 =0;
unsigned long lastTimeIn1 =0;
unsigned long lastTimeIn2 =0;
unsigned long lastTimeIn3 =0;
unsigned long currentTime =0;

int publishInterval = 5000;   //number of milliseconds for periodic publishing data logging events
int debounceDelay = 20;       //delay to ensure input signal debounce in milliseconds

void callback(char* inTopic, byte* inPayload, unsigned int length) {
 
  // Note: Do not use the client in the callback to publish, subscribe or
  // unsubscribe as it may cause deadlocks when other things arrive while
  // sending and receiving acknowledgments. Instead, change a global variable,
  // or push to a queue and handle it in the loop after calling `client.loop()`.

  if (strcmp(inTopic,inTopic0)==0){
   if (inPayload[0] == '1'){
      //out0State = true;    //force synchronization of the ledState variable with the MQTT switch
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
      //out1State = true;    //force synchronization of the ledState variable with the MQTT switch
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
      //out2State = true;    //force synchronization of the ledState variable with the MQTT switch
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
      //out3State = true;    //force synchronization of the ledState variable with the MQTT switch
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


void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("running setup");

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
 
  WiFi.begin(ssid, password);
  espClient.setCACert(test_root_ca);
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
 
    if (client.connect("ESP8266Client", mqttUser, mqttPassword )) {
       Serial.println("connected");  
     }else {
        Serial.print("failed with state ");
        Serial.print(client.state());
        delay(2000);
     }
  }
 
  client.publish("esp/test", "Hello from ESP32");
  client.subscribe("esp/test");

  //MQTT Subscriptions for control of digital outputs
  client.subscribe(inTopic0);
  client.subscribe(inTopic1);
  client.subscribe(inTopic2);
  client.subscribe(inTopic3);

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




void loop() {
  // put your main code here, to run repeatedly:

  client.loop();

  currentTime = millis();



  if(currentTime-lastTime >= publishInterval){
    if(tcToggle){
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
        if (client.publish(temp0Topic, (char*) outPayload.c_str())){
          Serial.println ("Publish ok");
          Serial.println(temp0Topic);
          lastTime = currentTime;
        }else {
          Serial.println("Publish failed");
        }
      }
    }else{
      //Serial.print("Toggle is working");
      digitalWrite(MAXCS1, LOW);
      double temp = thermocouple1.readCelsius();
      digitalWrite(MAXCS1, HIGH);
      if (isnan(temp)) {
        Serial.println("Something wrong with thermocouple0!");
      } else {
        Serial.print("Temp 1 = ");
        Serial.println(temp);
        Serial.println(" °C");
        //outTopic = "temp"; //temp0Topic;
        outPayload = String(temp);
        if (client.publish(temp1Topic, (char*) outPayload.c_str())){
          Serial.println ("Publish ok");
          Serial.println(temp1Topic);
          lastTime = currentTime;
        }else {
          Serial.println("Publish failed");
        }
      }      
    }
  tcToggle = !tcToggle;

   //Serial.print("F = ");
   //Serial.println(thermocouple.readFahrenheit());
  }



  // Resolve the hardware inputs
  in0Current = digitalRead(in0);
  if(in0Current != in0Previous){
    if (currentTime-lastTimeIn0 > debounceDelay){;    //debounce delay
      if(in0Current != in0Previous){
        if(in0Current == true){     //rising edge
          outPayload = "1";
          if (client.publish(outTopic0, (char*) outPayload.c_str())){
            Serial.print(outTopic0);
            Serial.println(": 1");
          }else {
              Serial.print("Publish failed");
          }
        }else{    //falling edge
          outPayload = "0";
          if (client.publish(outTopic0, (char*) outPayload.c_str())){
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
          if (client.publish(outTopic1, (char*) outPayload.c_str())){
            Serial.print(outTopic1);
            Serial.println(": 1");
          }else {
              Serial.println("Publish failed");
          }
        }else{    //falling edge
          outPayload = "0";
          if (client.publish(outTopic1, (char*) outPayload.c_str())){
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
          if (client.publish(outTopic2, (char*) outPayload.c_str())){
            Serial.print(outTopic2);
            Serial.println(": 1");
          }else {
              Serial.println("Publish failed");
          }
        }else{    //falling edge
          outPayload = "0";
          if (client.publish(outTopic2, (char*) outPayload.c_str())){
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
          if (client.publish(outTopic3, (char*) outPayload.c_str())){
            Serial.print(outTopic3);
            Serial.println(": 1");
          }else {
              Serial.println("Publish failed");
          }
        }else{    //falling edge
          outPayload = "0";
          if (client.publish(outTopic3, (char*) outPayload.c_str())){
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


  delay(100);
}