/*
 *******************************************************************************
 *
 * Purpose: Example of using the Arduino MqttClient with Esp8266WiFiClient.
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


#define LOG_PRINTFLN(fmt, ...)	logfln(fmt, ##__VA_ARGS__)
#define LOG_SIZE_MAX 128
void logfln(const char *fmt, ...) {
	char buf[LOG_SIZE_MAX];
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(buf, LOG_SIZE_MAX, fmt, ap);
	va_end(ap);
	Serial.println(buf);
}

#define HW_UART_SPEED									115200L
#define MQTT_ID											"TEST-ID"

#define WIFI_SSID  "YOUR_SSID"
#define WIFI_PWD   "YOUR_PASSWORD"

//#define MQTT_SERVER "test.mosquitto.org"
#define MQTT_SERVER "192.168.1.79"
//const char* MQTT_TOPIC_SUB = "#";
const char * MQTT_TOPIC_SUB= "lights/ambient/feedback";
const char* MQTT_TOPIC_PUB = "/STARTUP/";

static MqttClient *mqtt = NULL;
static WiFiClient network;
static bool subscribed=false;
static bool poweredUp = true;


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
}

// ============== Main loop ====================================================
void loop() {
	// Check connection status
	if (!mqtt->isConnected()) {
		// Close connection if exists
		network.stop();
		// Re-establish TCP connection with MQTT broker
		LOG_PRINTFLN("Connecting");
//    network.connect("test.mosquitto.org", 1883);
     network.connect(MQTT_SERVER, 1883);
     poweredUp = true;
  	
  	if (!network.connected()) {
			LOG_PRINTFLN("Can't establish the TCP connection");
			delay(5000);
			ESP.reset();
		}




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
    			// Add publish here if required
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
}  //Loop
