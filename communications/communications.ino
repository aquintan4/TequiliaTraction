#include <WiFi.h> // Wifi library
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <cstring>
#include <iostream>

// #include "secret.h"

//======== SERVER CONSTANTS =======

#define AIO_SERVER "193.147.79.118"
#define AIO_SERVERPORT 21883
#define TOPIC "/SETR/2024/7/"

#define TEAM_NAME "TequilaTraction"
#define TEAM_ID "7"

//======== MSG CONSTANTS ==========

#define START_LAP_ACTION "START_LAP"
#define END_LAP_ACTION "END_LAP"
#define OBSTACLE_DETECTED_ACTION "OBSTACLE_DETECTED"
#define LINE_LOST_ACTION "LINE_LOST"
#define PING_ACTION "PING"
#define INIT_LINE_SEARCH_ACTION "INIT_LINE_SEARCH"
#define END_LINE_SEARCH_ACTION "STOP_LINE_SEARCH"
#define LINE_FOUND_ACTION "LINE_FOUND"
#define VISIBLE_LINE "VISIBLE_LINE"

//===== SERIAL PORT CONSTANTS =====

#define RXD2 33
#define TXD2 4

// Enum to represent the type of values being sent (int, float, long)
enum ValueType {
    VALUE_INT,
    VALUE_FLOAT,
    VALUE_LONG
};

// Wifi credentials
const char* ssid = "something"; // Wifi SSID
const char* pwd = "also_something"; // Wifi password

// Base message template
String base_message = "\n{\n\t\"team_name\": \"" + String(TEAM_NAME) + "\",\n\t\"id\": " + String(TEAM_ID) + ",\n\t\"action\": ";

WiFiClient client; // WiFi client instance
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT); // MQTT client instance
Adafruit_MQTT_Publish pub = Adafruit_MQTT_Publish(&mqtt, TOPIC); // MQTT publish instance

// Function to publish a message to the MQTT server
void publish_msg(const String *msg, Adafruit_MQTT_Publish* publisher) {
  if (!publisher->publish(msg->c_str(), TOPIC)) { // Publish and check for success
    Serial.println("Error al publicar"); // Print error if failed
  }
}

// Function to send operation message (action only)
void send_operation(String *action) {
  String operation_msg = base_message + "\"" + *action + "\"" + "\n}"; // Build message
  publish_msg(&operation_msg, &pub); // Publish the message
}

// Function to send operation message with additional value
void send_operation_and_value(String *action, const char* name_value_field, void* value, ValueType type) {
    String operation_msg = base_message + "\"" + *action + "\",\n"; // Start message
    operation_msg += "\t\"" + String(name_value_field) + "\": ";

    // Add the appropriate value based on its type
    if (type == VALUE_INT) {
        operation_msg += String(*((int*)value)); // Convert int to string
    } else if (type == VALUE_FLOAT) {
        operation_msg += String(*((float*)value), 2); // Convert float to string with 2 decimal points
    } else if (type == VALUE_LONG) {
        operation_msg += String(*((long*)value)); // Convert long to string
    }
  
    operation_msg += "\n}"; // End message
    publish_msg(&operation_msg, &pub); // Publish the message
}

void setup() {
  int ret;

  Serial.begin(115200); // Start serial communication for debugging
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); // Start second serial port for communication
  delay(10); // Short delay

  // Print WiFi connection status
  Serial.print(F("Connecting to network: "));
  Serial.println(ssid);
  WiFi.disconnect(true); // Disconnect from previous WiFi connections
  WiFi.begin(ssid, pwd); // Connect to the WiFi network

  while (WiFi.status() != WL_CONNECTED) { // Wait for WiFi connection
    delay(500); 
    Serial.print(F("."));
  }

  while (!mqtt.connected()) { // Wait for MQTT connection
    if ((ret = mqtt.connect())) { // Try to connect to MQTT
      Serial.println(mqtt.connectErrorString(ret)); // Print connection error
      mqtt.disconnect(); // Disconnect if failed
      delay(5000); // Retry after a delay
    }
  }

  while (1) { // Wait for command from serial
    Serial2.print("#commReady_");
    if (Serial2.available() > 0) {
      String msg = Serial2.readStringUntil('_'); // Read until underscore
      if (msg == "#ACK") { // Wait for ACK response
        break; // Exit loop when ACK is received
      }
    }
  }
}

// Function to execute the operation based on the received message
void execute_operation(String* msg_recieved) {
  // Validate message format
  if (!msg_recieved->startsWith("#") && !msg_recieved->endsWith("_")) {
    return; // Invalid message format
  }

  // Clean message format
  *msg_recieved = msg_recieved->substring(1, msg_recieved->length() - 2);
  int colon_index = msg_recieved->indexOf(':'); // Find colon in message

  String action = colon_index != -1 ? msg_recieved->substring(0, colon_index) : *msg_recieved; // Extract action
  String value = colon_index != -1 ? msg_recieved->substring(colon_index + 1) : ""; // Extract value

  // Handle different actions
  if (action == PING_ACTION) {
    int time = value.toInt(); // Convert value to int
    send_operation_and_value(&action, "time", &time, VALUE_LONG); // Send operation with value
    return;
  }

  if (action == OBSTACLE_DETECTED_ACTION) {
    float distance = value.toFloat(); // Convert value to float
    send_operation_and_value(&action, "distance", &distance, VALUE_FLOAT); // Send operation with value
    return;
  }

  if (action == LINE_LOST_ACTION || action == LINE_FOUND_ACTION || action == INIT_LINE_SEARCH_ACTION || action == END_LINE_SEARCH_ACTION) {
    send_operation(&action); // Send operation without value
    return;
  }

  if (action == VISIBLE_LINE) {
    float percentage = value.toFloat(); // Convert value to float
    send_operation_and_value(&action, "value", &percentage, VALUE_FLOAT); // Send operation with value
    return;
  }

  if (action == START_LAP_ACTION) {
    send_operation(&action); // Send start lap operation
    return;
  }

  if (action == END_LAP_ACTION) {
    int time = value.toInt(); // Convert value to int
    send_operation_and_value(&action, "time", &time, VALUE_LONG); // Send end lap operation with time
    return;
  }
}

void loop() {
  if (Serial2.available() > 0) { // Check if there's data to read from serial
    String msg = Serial2.readStringUntil('\n');  // Read message
    execute_operation(&msg); // Execute the operation based on received message
  }
}
