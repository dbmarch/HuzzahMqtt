/*
 * 
 * This client will join a wifi network from a list of wifi SSID's.
 * From there it will look for an MQTT broker trying a predefined list - and ultimately trying every IP address.
 * 
 * Once it connects to the broker it will generate:
 * 1) Power Up message
 * 2) Ambient Light status
 * 3) LightFeedback Status
 * 4) Power Fail message
 * 
 * It will subscribe to:
 * 1) Light Control 
 * 2)
 *******************************************************************************
 *
 * Started from an example of using the Arduino MqttClient with Esp8266WiFiClient.
 * Project URL: https://github.com/monstrenyatko/ArduinoMqtt
 *
 *******************************************************************************
 * Copyright Oleg Kovalenko 2017.
 *
 * Distributed under the MIT License.
 * (See accompanying file LICENSE or copy at http://opensource.org/licenses/MIT)
 *******************************************************************************
 */
#include <Arduino.h>
#include <ESP8266WiFi.h>

// Enable MqttClient logs
#define MQTT_LOG_ENABLED 1
// Include library
#include <MqttClient.h>


#define LOG_PRINTFLN(fmt, ...)  logfln(fmt, ##__VA_ARGS__)
#define LOG_SIZE_MAX 128
void logfln(const char *fmt, ...) {
        char buf[LOG_SIZE_MAX];
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(buf, LOG_SIZE_MAX, fmt, ap);
        va_end(ap);
        Serial.println(buf);
}

#define LED_RED   0
#define LED_BLUE  2
#define PIN_RELAY       14
#define PIN_POWERDETECT 12
#define PIN_LIGHTSTATE  13


#define HW_UART_SPEED   115200L

// convert to using a port pin for identification
#define UNIT    1

#if UNIT == 1
#define MQTT_ID         "lamp-001"
#else
#define MQTT_ID         "lamp-002"
#endif

struct wifiLogin {
  const char * ssid;
  const char * pass;
} ;


#include "wifiList.h"
#ifndef __WIFILIST_H_
//  Create this structure here or put in the wifiList.h file.
static const struct wifiLogin wifi [] = 
{
  {  "ssid_1", "password_ssid_1"},
  {  "ssid_2", "password_ssid_2"},
  {  "ssid_3", "password_ssid_3"}
};
#endif

#define NUM_WIFI (sizeof (wifi) / sizeof (wifiLogin))

static const char * mqttBroker[] = 
{
  "thejoveexpress.local",
  "pihub",
  "192.168.1.240"
};
#define NUM_MQTT_BROKER (sizeof(mqttBroker) / sizeof(char*) )

//Subscriptions:
#define MAX_SUBSCRIPTIONS    3
const char* MQTT_TOPIC_FEEDBACK_SUB = MQTT_ID "/feedback";
const char* MQTT_TOPIC_POWER_SUB    = MQTT_ID "/power/control";
const char* MQTT_TOPIC_JOVEXPRESS_FAULT_SUB = "thejoveexpress/fault";

//Publications:
const char* MQTT_TOPIC_LIFECYCLE    = MQTT_ID "/lifecycle/feedback";
const char* MQTT_TOPIC_POWERUP      = MQTT_ID "/powerup/feedback";
const char* MQTT_TOPIC_AMBIENT      = MQTT_ID "/ambient/feedback";
const char* MQTT_TOPIC_LIGHTFEEDBACK = MQTT_ID "/light/feedback";
const char* MQTT_TOPIC_POWER         = MQTT_ID "/power/feedback";
const char* MQTT_TOPIC_FAULT_POWER   = MQTT_ID "/powerfail";
const char* MQTT_TOPIC_FAULT_BULB    = MQTT_ID "/bulbfault";


static MqttClient *mqtt = NULL;
static WiFiClient network;
static bool subscribed=false;
static bool powerupSent = false;
static bool online = false;
int    wifiIndex = 0;
int    redState = HIGH;

static int  ambientLevel = 0;
static bool powerSetting = false;
static bool lightState = false;
static bool powerDetect = false;
static bool bulbFault = false;

void publishPowerFeedbackMessage(void);
void publishAmbientMessage (void);
void publishLightStateMessage ( void);
void publishPowerupMessage(void);
void publishPowerFaultMessage(void);
void publishBulbFaultMessage (void);
void publishLifecycle (void);


// ============== Object to supply system functions ============================
class System: public MqttClient::System {
public:
        unsigned long millis() const {
                return ::millis();
        }

        void yield(void) {
                ::yield();
        }
};                                                                        

void blinkRed(int cnt) {
   redState = (redState == HIGH ? LOW : HIGH);
   digitalWrite (LED_RED, redState);
   for (int i = 0; i < cnt; i++) {
      delay(100);
      redState = (redState == HIGH ? LOW : HIGH);
      digitalWrite (LED_RED, redState);
      delay(100);
      redState = (redState == HIGH ? LOW : HIGH);
      digitalWrite (LED_RED, redState);
   }
    delay(100);
    redState = (redState == HIGH ? LOW : HIGH);
    digitalWrite (LED_RED, redState);
}

// ============== Setup all objects ============================================
void setup() {
  pinMode (LED_RED, OUTPUT);
  pinMode (LED_BLUE, OUTPUT);
  pinMode (PIN_RELAY, OUTPUT);
  pinMode (PIN_LIGHTSTATE, INPUT);
  pinMode (PIN_POWERDETECT, INPUT);
  
  digitalWrite(LED_RED, redState);       // sets the digital pin 13 on
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite (PIN_RELAY, LOW);
  lightState   = digitalRead (PIN_LIGHTSTATE)== LOW ? true : false;  
  powerDetect  = digitalRead (PIN_POWERDETECT) == LOW ? false : true;
  
  // Setup hardware serial for logging
  Serial.begin(HW_UART_SPEED);
  while (!Serial);

  LOG_PRINTFLN("\n");

  LOG_PRINTFLN( "Number Brokers  = %d", NUM_MQTT_BROKER);
  for (int i = 0; i < NUM_MQTT_BROKER; i++)
     LOG_PRINTFLN ("mqttBroker[%d]: %s", i, mqttBroker[i]);

  LOG_PRINTFLN( "Number Wifi = %d", NUM_WIFI);
  for (int i = 0; i < NUM_WIFI; i++)
     LOG_PRINTFLN ("wifi[%d]: %s", i, wifi[i].ssid);
     
  // Setup WiFi network
  WiFi.mode(WIFI_STA);
  WiFi.hostname(MQTT_ID);

  while (!online) {
     WiFi.begin (wifi[wifiIndex].ssid, wifi [wifiIndex].pass);

     LOG_PRINTFLN("Connecting to WiFi");
     LOG_PRINTFLN(wifi[wifiIndex].ssid);
     LOG_PRINTFLN(wifi[wifiIndex].pass);

     for (int i = 0; i < 20; i++) {
       redState = (redState == HIGH ? LOW : HIGH);
       digitalWrite (LED_RED,redState);
       
       if (WiFi.status() == WL_CONNECTED) {
          digitalWrite (LED_BLUE, LOW);
          online = true;
          LOG_PRINTFLN("CONNECTED!");
          break;
       } else {
          LOG_PRINTFLN(".");
          delay(500); 
       }
     }

     if (!online) {
       LOG_PRINTFLN ("TRY OTHER NETWORK");
       digitalWrite (LED_BLUE, LOW);
       delay (1000);
       digitalWrite (LED_BLUE, HIGH);
       wifiIndex++;
       if (wifiIndex >= NUM_WIFI)
          wifiIndex = 0;
     } 
  }
  
  digitalWrite (LED_BLUE, LOW);
  LOG_PRINTFLN("Connected to WiFi");
  LOG_PRINTFLN("IP: %s", WiFi.localIP().toString().c_str());
  LOG_PRINTFLN("GATEWAY: %s", WiFi.gatewayIP().toString().c_str());

  // Setup MqttClient
  MqttClient::System *mqttSystem = new System;
  MqttClient::Logger *mqttLogger = new MqttClient::LoggerImpl<HardwareSerial>(Serial);
  MqttClient::Network * mqttNetwork = new MqttClient::NetworkClientImpl<WiFiClient>(network, *mqttSystem);
  //// Make 128 bytes send buffer
  MqttClient::Buffer *mqttSendBuffer = new MqttClient::ArrayBuffer<128>();
  //// Make 128 bytes receive buffer
  MqttClient::Buffer *mqttRecvBuffer = new MqttClient::ArrayBuffer<128>();
  //// Allow up to 2 subscriptions simultaneously
  MqttClient::MessageHandlers *mqttMessageHandlers = new MqttClient::MessageHandlersImpl<MAX_SUBSCRIPTIONS>();
  //// Configure client options
  MqttClient::Options mqttOptions;
  ////// Set command timeout to 10 seconds
  mqttOptions.commandTimeoutMs = 10000;
  //// Make client object
  mqtt = new MqttClient(
          mqttOptions, *mqttLogger, *mqttSystem, *mqttNetwork, *mqttSendBuffer,
          *mqttRecvBuffer, *mqttMessageHandlers
  );
}                                                  

// ============== Subscription callback ========================================
void processPowerMessage(MqttClient::MessageData& md) {
  const MqttClient::Message& msg = md.message;
  char payload[msg.payloadLen + 1];
  memcpy(payload, msg.payload, msg.payloadLen);
  payload[msg.payloadLen] = '\0';
  LOG_PRINTFLN ("Received Power Control: %d", payload[0]);
  LOG_PRINTFLN ("PayloadLen = %d", msg.payloadLen);
  LOG_PRINTFLN("Message arrived: qos %d, retained %d, dup %d, packetid %d, payload:[%s]",
    msg.qos, msg.retained, msg.dup, msg.id, payload
  );  

  powerSetting = (payload[0] == 0) ? false : true;
  if (powerSetting)
      digitalWrite (PIN_RELAY, HIGH);
  else
      digitalWrite (PIN_RELAY, LOW);

  publishPowerFeedbackMessage();

  blinkRed(2);
}

void processFeedbackMessage(MqttClient::MessageData& md) {
  const MqttClient::Message& msg = md.message;
//  memcpy(payload, msg.payload, msg.payloadLen);
//  payload[msg.payloadLen] = '\0';
  LOG_PRINTFLN ("Received Feedback");
  LOG_PRINTFLN ("PayloadLen = %d", msg.payloadLen);
  LOG_PRINTFLN( "Message arrived: qos %d, retained %d, dup %d, packetid %d",
    msg.qos, msg.retained, msg.dup, msg.id );  

  // There are no args expected.  Now we are to publish everything about our device:
  publishPowerupMessage();
  publishPowerFeedbackMessage();
  publishLightStateMessage();  
  publishAmbientMessage();
  publishBulbFaultMessage();
  publishPowerFaultMessage();
  blinkRed(3);
}

// ============== Publish Messages  ========================================
void publishAmbientMessage (void) {
      LOG_PRINTFLN("Sending ambient status: %04X", ambientLevel);
      char buf[10];
      buf[0] = (uint8_t)((ambientLevel & 0xFF000000) >> 24);
      buf[1] = (uint8_t)((ambientLevel & 0x00FF0000) >> 16);
      buf[2] = (uint8_t)((ambientLevel & 0x0000FF00) >> 8 );
      buf[3] = (uint8_t)((ambientLevel & 0x000000FF)) ;
      LOG_PRINTFLN("Bytes = %02X %02X %02X %02X", buf[0], buf[1], buf[2], buf[3]);

      MqttClient::Message message;
      message.qos = MqttClient::QOS0;
      message.retained = false;
      message.dup = false;
      message.payload = (void*) buf;
      message.payloadLen = 4;
      mqtt->publish(MQTT_TOPIC_AMBIENT, message);
      blinkRed(1);
}

void publishPowerupMessage(void) {
    LOG_PRINTFLN("Sending Hello Message");
                  // Add publish here if required
    char buf[30];
    strcpy (buf, WiFi.localIP().toString().c_str());
    buf[WiFi.localIP().toString().length()] = 0;
    MqttClient::Message message;
    message.qos = MqttClient::QOS0;
    message.retained = false;
    message.dup = false;
    message.payload = (void*) buf;
    message.payloadLen = strlen(buf);   // Don't include the NULL in the message.
    mqtt->publish(MQTT_TOPIC_POWERUP, message);
    blinkRed(1);
}

void publishLifecycleMessage(void) {
    LOG_PRINTFLN("Sending lifecycle Message");
    uint8_t  buf[10];
    buf[0] = 1;
    MqttClient::Message message;
    message.qos = MqttClient::QOS0;
    message.retained = false;
    message.dup = false;
    message.payload = (void*) buf;
    message.payloadLen = 1;   
    mqtt->publish(MQTT_TOPIC_LIFECYCLE, message);
    blinkRed(1);
}

void publishPowerFeedbackMessage(void) {
      uint8_t buf = powerSetting ? 1 : 0;
      LOG_PRINTFLN("Sending Power: %02X", buf);
      MqttClient::Message message;
      message.qos = MqttClient::QOS0;
      message.retained = false;
      message.dup = false;
      message.payload = (void*) &buf;
      message.payloadLen = 1;
      mqtt->publish(MQTT_TOPIC_POWER, message);
      blinkRed(1);
}

void publishLightStateMessage(void) {
      uint8_t buf = lightState ? 1 : 0;
      LOG_PRINTFLN("Sending LightFeedback status: %02X", buf);
      MqttClient::Message message;
      message.qos = MqttClient::QOS0;
      message.retained = false;
      message.dup = false;
      message.payload = (void*) &buf;
      message.payloadLen = 1;
      mqtt->publish(MQTT_TOPIC_LIGHTFEEDBACK, message);
      blinkRed(1);
}

void publishPowerFaultMessage(void) {
      uint8_t buf = powerDetect ? 0 : 1;   // Reverse.   If we detect power, there is no fault.
      LOG_PRINTFLN("Sending Power Fault: %02X", buf);
      MqttClient::Message message;
      message.qos = MqttClient::QOS0;
      message.retained = false;
      message.dup = false;
      message.payload = (void*) &buf;
      message.payloadLen = 1;
      mqtt->publish(MQTT_TOPIC_FAULT_POWER, message);
      blinkRed(1);
}

void publishBulbFaultMessage (void) {
      uint8_t buf = bulbFault ? 1 : 0;   
      LOG_PRINTFLN("Sending Bulb Fault: %02X", buf);
      MqttClient::Message message;
      message.qos = MqttClient::QOS0;
      message.retained = false;
      message.dup = false;
      message.payload = (void*) &buf;
      message.payloadLen = 1;
      mqtt->publish(MQTT_TOPIC_FAULT_BULB, message);
      blinkRed(1);

}

// ============== subscribeBroker ====================================================
void subscribeBroker( void ) {
    if (!subscribed) {
      // Add subscribe here if required
      MqttClient::Error::type rc = mqtt->subscribe(
          MQTT_TOPIC_POWER_SUB, MqttClient::QOS0, processPowerMessage
      );
      LOG_PRINTFLN ("Subscribing to %s", MQTT_TOPIC_POWER_SUB);

     rc = mqtt->subscribe(
          MQTT_TOPIC_FEEDBACK_SUB, MqttClient::QOS0, processFeedbackMessage
      );
      LOG_PRINTFLN ("Subscribing to %s", MQTT_TOPIC_FEEDBACK_SUB);
        
      subscribed = true;
      if (rc != MqttClient::Error::SUCCESS) {
         LOG_PRINTFLN("Subscribe error: %i", rc);
         subscribed = false;
         return;
      }  // Error
    }  // ! subscribed
}

// ============== Connect Broker ====================================================
bool connectBrokerSocket ( void ) {
  MqttClient::ConnectResult connectResult;
  if (!mqtt->isConnected()) {
     // Close connection if exists
     network.stop();
     // Re-establish TCP connection with MQTT broker
     powerupSent = false;

     for (int i = 0; i < NUM_MQTT_BROKER; i++) {
       LOG_PRINTFLN("Connecting to BROKER: %s", mqttBroker[i]);
       network.connect(mqttBroker [i], 1883);
       if (network.connected()){
          LOG_PRINTFLN("Connected to BROKER: %s", mqttBroker[i]);
          break;
       }
       else
          blinkRed(5);
     }
     if (!network.connected()) {
       LOG_PRINTFLN("Attempting to find Broker @ Gateway IP");
       network.connect(WiFi.gatewayIP(), 1883);
       LOG_PRINTFLN ("Found Broker @ Gateway IP");
     }
    
    if (!network.connected()) {
      LOG_PRINTFLN("Can't establish the TCP connection to BROKER");
      blinkRed(5);
       // Let's scan the network for brokers:
        uint32_t addr = WiFi.gatewayIP();
        unsigned char *ipBytes= (unsigned char*) &addr;
        // Typically gateway IP Address ends in x.x.x.1
        LOG_PRINTFLN("Gateway = %d.%d.%d.%d",ipBytes[0], ipBytes[1], ipBytes[2], ipBytes[3]); 
        for (int i = 1; i < 254; i++) {
            ipBytes[3] = i;
            LOG_PRINTFLN("Attempting Broker @ %d.%d.%d.%d",ipBytes[0], ipBytes[1], ipBytes[2], ipBytes[3]); 
            network.connect(addr, 1883);
            if (network.connected()) {
                LOG_PRINTFLN("Connected to BROKER!");
                break;
            }
        }
    }
    if (!network.connected()) {
       delay(5000);    // Try again in 5 seconds....
       return (false);
       //ESP.reset();
    } else {
       LOG_PRINTFLN ("TCP CONNECTION to broker established");
    }

   redState = LOW;
   digitalWrite (LED_RED, redState); /* Set RED LED ON */

   MQTTPacket_connectData options = MQTTPacket_connectData_initializer;
   options.MQTTVersion = 4;
   options.clientID.cstring = (char*)MQTT_ID;
   options.cleansession = true;
   options.keepAliveInterval = 15; // 15 seconds
//   options.willFlag 
   options.will = { {'M', 'Q', 'T', 'W'}, 0, {NULL, {0, NULL}}, {NULL, {0, NULL}}, 0, 0 };
//   options.will.struct_id = "MQTW";
//   options.will.struct_version = 0;
//   options.will.topicName = // MQTTstring;
//   options.will.message   = // MQTTstring
//   options.will.qos = //
      
   MqttClient::Error::type rc = mqtt->connect(options, connectResult);
   if (rc != MqttClient::Error::SUCCESS) {
      LOG_PRINTFLN("Connection error: %i", rc);
      // should turn off red led and disconnect from network...
      return false;
   }
   subscribeBroker();
   return (true);
  }   // ! connnected
}

// ============== Main loop ====================================================
void loop() {
    // Start new MQTT connection
    // Check connection status
    if (!connectBrokerSocket())
       return;
 
  if (!powerupSent)  {
     int lightState    =  digitalRead ( PIN_LIGHTSTATE ) == LOW ? true : false;
     int powerDetect   = digitalRead ( PIN_POWERDETECT ) == LOW ? false : true;
     int ambientLevel  = analogRead (A0);

     LOG_PRINTFLN("LightState = %d", lightState);
     LOG_PRINTFLN ("powerDetect = %d", powerDetect);

     publishLifecycleMessage();   
     publishPowerFeedbackMessage();
     publishPowerupMessage();     
     publishLightStateMessage();
     publishPowerFaultMessage();
     
     if (! bulbFault && powerDetect && powerSetting  && !lightState) {
         bulbFault = true;
     }
     publishBulbFaultMessage();
     powerupSent = true;
  } else {
     int currentLight = digitalRead ( PIN_LIGHTSTATE ) == LOW ? true : false;
     int currentPower = digitalRead ( PIN_POWERDETECT ) == LOW ? false : true;
     int currentAmbient = analogRead (A0);

     if (currentLight != lightState) {
        lightState = currentLight;;
        publishLightStateMessage();  
     }
     
     if (currentPower != powerDetect) {
        powerDetect = currentPower;
        publishPowerFaultMessage();
     }

     if (currentAmbient != ambientLevel) {
        ambientLevel = currentAmbient;
        publishAmbientMessage();
     }
     if (! bulbFault && powerDetect && powerSetting  && !lightState) {
         bulbFault = true;
         publishBulbFaultMessage();
     } else if (bulbFault && powerDetect && powerSetting && lightState ) {
         bulbFault = false;
         publishBulbFaultMessage();
     }
   }
  // Idle for 250msec
  mqtt->yield(250L);
}  //Loop

