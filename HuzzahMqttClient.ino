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

<<<<<<< HEAD
#define LED_RED   0
#define LED_BLUE  2

#define HW_UART_SPEED									115200L
#define MQTT_ID   								    "lamp-001"

struct wifiLogin {
  const char * ssid;
  const char * pass;
} ;


=======
#define HW_UART_SPEED 115200L
#define MQTT_ID       "TEST-ID"
>>>>>>> 2e4a6224bc1c20cfbe35c435610d827fa3af998b

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
  "pihub"
};
#define NUM_MQTT_BROKER (sizeof(mqttBroker) / sizeof(char*) )

const char* MQTT_TOPIC_POWERUP      = MQTT_ID "/powerup";
const char* MQTT_TOPIC_POWER        = MQTT_ID "/power";
const char* MQTT_TOPIC_AMBIENT      = MQTT_ID "/ambient";
const char* MQTT_TOPIC_LAMPFEEDBACK = MQTT_ID "/lampfeedback";

static MqttClient *mqtt = NULL;
static WiFiClient network;
static bool subscribed=false;
static bool poweredUp = true;
static bool online = false;
int    wifiIndex = 0;
int    redState = HIGH;

int  ambientLevel = 0;



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

<<<<<<< HEAD
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

  digitalWrite(LED_RED, redState);       // sets the digital pin 13 on
  digitalWrite(LED_BLUE, HIGH);
  
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
	MqttClient::MessageHandlers *mqttMessageHandlers = new MqttClient::MessageHandlersImpl<2>();
	//// Configure client options
	MqttClient::Options mqttOptions;
	////// Set command timeout to 10 seconds
	mqttOptions.commandTimeoutMs = 10000;
	//// Make client object
	mqtt = new MqttClient(
		mqttOptions, *mqttLogger, *mqttSystem, *mqttNetwork, *mqttSendBuffer,
		*mqttRecvBuffer, *mqttMessageHandlers
	);
=======
                                                  
// ============== Setup all objects ============================================
void setup() {
        // Setup hardware serial for logging
        Serial.begin(HW_UART_SPEED);
        while (!Serial);

        // Setup WiFi network
        WiFi.mode(WIFI_STA);
        WiFi.hostname("ESP_" MQTT_ID);
        WiFi.begin(WIFI_SSID, WIFI_PWD);
        LOG_PRINTFLN("\n");
        LOG_PRINTFLN("Connecting to WiFi");
        while (WiFi.status() != WL_CONNECTED) {
                delay(500);
                LOG_PRINTFLN(".");
        }
        LOG_PRINTFLN("Connected to WiFi");
        LOG_PRINTFLN("IP: %s", WiFi.localIP().toString().c_str());

        // Setup MqttClient
        MqttClient::System *mqttSystem = new System;
        MqttClient::Logger *mqttLogger = new MqttClient::LoggerImpl<HardwareSerial>(Serial);
        MqttClient::Network * mqttNetwork = new MqttClient::NetworkClientImpl<WiFiClient>(network, *mqttSystem);
        //// Make 128 bytes send buffer
        MqttClient::Buffer *mqttSendBuffer = new MqttClient::ArrayBuffer<128>();
        //// Make 128 bytes receive buffer
        MqttClient::Buffer *mqttRecvBuffer = new MqttClient::ArrayBuffer<128>();
        //// Allow up to 2 subscriptions simultaneously
        MqttClient::MessageHandlers *mqttMessageHandlers = new MqttClient::MessageHandlersImpl<2>();
        //// Configure client options
        MqttClient::Options mqttOptions;
        ////// Set command timeout to 10 seconds
        mqttOptions.commandTimeoutMs = 10000;
        //// Make client object
        mqtt = new MqttClient(
                mqttOptions, *mqttLogger, *mqttSystem, *mqttNetwork, *mqttSendBuffer,
                *mqttRecvBuffer, *mqttMessageHandlers
        );
>>>>>>> 2e4a6224bc1c20cfbe35c435610d827fa3af998b
}


// ============== Subscription callback ========================================
void processMessage(MqttClient::MessageData& md) {
  const MqttClient::Message& msg = md.message;
  char payload[msg.payloadLen + 1];
  memcpy(payload, msg.payload, msg.payloadLen);
  payload[msg.payloadLen] = '\0';
  LOG_PRINTFLN(
    "Message arrived: qos %d, retained %d, dup %d, packetid %d, payload:[%s]",
    msg.qos, msg.retained, msg.dup, msg.id, payload
  );
  blinkRed(2);

}

// ============== Main loop ====================================================
void loop() {
<<<<<<< HEAD
	// Check connection status

	if (!mqtt->isConnected()) {
    
		// Close connection if exists
		network.stop();
		// Re-establish TCP connection with MQTT broker
    poweredUp = true;

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
       return;
       //ESP.reset();
	  } else {
       LOG_PRINTFLN ("TCP CONNECTION to broker established");
  	}
    redState = LOW;
    digitalWrite (LED_RED, redState); /* Set RED LED ON */
     
=======
  // Check connection status
   if (!mqtt->isConnected()) {
      // Close connection if exists
      network.stop();
      // Re-establish TCP connection with MQTT broker
      LOG_PRINTFLN("Connecting");
//   network.connect("test.mosquitto.org", 1883);
     network.connect(MQTT_SERVER, 1883);
     poweredUp = true;
        
      if (!network.connected()) {
          LOG_PRINTFLN("Can't establish the TCP connection");
          delay(5000);
          ESP.reset();
      }
>>>>>>> 2e4a6224bc1c20cfbe35c435610d827fa3af998b
    // Start new MQTT connection
    MqttClient::ConnectResult connectResult;
    
    // Connect
    {
        MQTTPacket_connectData options = MQTTPacket_connectData_initializer;
        options.MQTTVersion = 4;
        options.clientID.cstring = (char*)MQTT_ID;
        options.cleansession = true;
        options.keepAliveInterval = 15; // 15 seconds
        MqttClient::Error::type rc = mqtt->connect(options, connectResult);
        if (rc != MqttClient::Error::SUCCESS) {
                LOG_PRINTFLN("Connection error: %i", rc);
                return;
        }
    }


    if (!subscribed) {
<<<<<<< HEAD
    // Add subscribe here if required
      MqttClient::Error::type rc = mqtt->subscribe(
        MQTT_TOPIC_POWER, MqttClient::QOS0, processMessage
        );

        LOG_PRINTFLN ("Subscribing to %s", MQTT_TOPIC_POWER);
        
       subscribed = true;
       if (rc != MqttClient::Error::SUCCESS) {
        LOG_PRINTFLN("Subscribe error: %i", rc);
        //  LOG_PRINTFLN("Drop connection");
        //  mqtt->disconnect();
        subscribed = false;
        return;
       }  // Error
    }  // ! subscribed
  }
		  if (poweredUp)  {
          LOG_PRINTFLN("Sending Hello Message");
    			// Add publish here if required
          const char* buf = "Hello";
          MqttClient::Message message;
          message.qos = MqttClient::QOS0;
          message.retained = false;
          message.dup = false;
          message.payload = (void*) buf;
          message.payloadLen = strlen(buf) + 1;
          mqtt->publish(MQTT_TOPIC_POWERUP, message);
          blinkRed(1);
          poweredUp = false;  
    	  	}   // Powered up
        else
        {
          LOG_PRINTFLN("Sending ambient status");
          // Add publish here if required
          int32_t value = ambientLevel;
          char * buf = "1";
          MqttClient::Message message;
          message.qos = MqttClient::QOS0;
          message.retained = false;
          message.dup = false;
          message.payload = (void*) &value;
          message.payloadLen = sizeof (value);
//           message.payload = (void*) buf;
//           message.payloadLen = strlen(buf) + 1;
           mqtt->publish(MQTT_TOPIC_AMBIENT, message);
          blinkRed(1);
          ambientLevel++;
          if (ambientLevel > 255)  
             ambientLevel = 0;
        }
		// Idle for 250msec
		mqtt->yield(250L);
   
=======
       // Add subscribe here if required
       MqttClient::Error::type rc = mqtt->subscribe(
           MQTT_TOPIC_SUB, MqttClient::QOS0, processMessage
       );
       subscribed = true;
       if (rc != MqttClient::Error::SUCCESS) {
           LOG_PRINTFLN("Subscribe error: %i", rc);
           LOG_PRINTFLN("Drop connection");
           //  mqtt->disconnect();
           subscribed = false;
           return;
       }  // Error
    }  // ! subscribed
    
    if (poweredUp)  {
         LOG_PRINTFLN("Sending Hello Message");
         const char* buf = "Hello";
         MqttClient::Message message;
         message.qos = MqttClient::QOS0;
         message.retained = false;
         message.dup = false;
         message.payload = (void*) buf;
         message.payloadLen = strlen(buf) + 1;
         mqtt->publish(MQTT_TOPIC_PUB, message);
         poweredUp = false;  
    }   // Powered up
    // Idle for 30 seconds
    mqtt->yield(30000L);
  }  //MQTT Connected
>>>>>>> 2e4a6224bc1c20cfbe35c435610d827fa3af998b
}  //Loop
