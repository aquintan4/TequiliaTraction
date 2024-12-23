#include <WiFi.h> // Wifi library
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <cstring>
#include <iostream>

// #include "secret.h"

#define AIO_SERVER "193.147.79.118"
#define AIO_SERVERPORT 21883
#define TOPIC "/SETR/2024/7/"

#define TEAM_NAME "TequilaTraction"
#define TEAM_ID "7"

#define START_LAP_ACTION "START_LAP"
#define END_LAP_ACTION "END_LAP"
#define OBSTACLE_DETECTED_ACTION "OBSTACLE_DETECTED"
#define LINE_LOST_ACTION "LINE_LOST"
#define PING_ACTION "PING"
#define INIT_LINE_SEARCH_ACTION "INIT_LINE_SEARCH"
#define END_LINE_SEARCH_ACTION "STOP_LINE_SEARCH"
#define LINE_FOUND_ACTION "LINE_FOUND"
#define VISIBLE_LINE "VISIBLE_LINE"

#define RXD2 33
#define TXD2 4

enum ValueType {
    VALUE_INT,
    VALUE_FLOAT,
    VALUE_LONG
};

// SSID NAME
const char* ssid = "POCO M3"; // eduroam SSID
const char* pwd = "pepepepe";

String base_message = "\n{\n\t\"team_name\": \"" + String(TEAM_NAME) + "\",\n\t\"id\": " + String(TEAM_ID) + ",\n\t\"action\": ";

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT);
Adafruit_MQTT_Publish pub = Adafruit_MQTT_Publish(&mqtt, TOPIC);

void publish_msg(const String *msg, Adafruit_MQTT_Publish* publisher) {
  if (!publisher->publish(msg->c_str(), TOPIC)) {
    Serial.println("Error al publicar");
  }
}

void send_operation(String *action) {
  String operation_msg;
  operation_msg = base_message + "\"" + *action + "\"" + "\n}";
  publish_msg(&operation_msg, &pub);
}

void send_operation_and_value(String *action, const char* name_value_field, void* value, ValueType type) {
    String operation_msg = base_message + "\"" + *action + "\",\n";
    operation_msg += "\t\"" + String(name_value_field) + "\": ";

    if (type == VALUE_INT) {
        operation_msg += String(*((int*)value)); // Convertir int
    } else if (type == VALUE_FLOAT) {
        operation_msg += String(*((float*)value), 2); // Convertir float
    } else if (type == VALUE_LONG) {
        operation_msg += String(*((long*)value)); // Convertir float
    }
  
    operation_msg += "\n}";
    publish_msg(&operation_msg, &pub);
}

void setup() {
  int ret;

  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  delay(10);
  Serial.print(F("Connecting to network: "));
  Serial.println(ssid);
  WiFi.disconnect(true); 

  // WiFi.begin(ssid, WPA2_AUTH_PEAP, EAP_IDENTITY, EAP_USERNAME, EAP_PASSWORD); 
  WiFi.begin(ssid, pwd); 

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }

  while (!mqtt.connected()) {
    if ((ret = mqtt.connect())) {
      Serial.println(mqtt.connectErrorString(ret));
      mqtt.disconnect();
      delay(5000);
    }
  }

  while (1) {
    Serial2.print("#commReady_");
    if (Serial2.available() > 0) {
      String msg = Serial2.readStringUntil('_');
      if (msg == "#ACK") {

        break;
      }
    }
  }
}

void execute_operation(String* msg_recieved) {
  // Validar formato del mensaje
  if (!msg_recieved->startsWith("#") && !msg_recieved->endsWith("_")) {
    return;
  }

  *msg_recieved = msg_recieved->substring(1, msg_recieved->length() - 2);
  int colon_index = msg_recieved->indexOf(':');

  String action = colon_index != -1 ? msg_recieved->substring(0, colon_index) : *msg_recieved;
  String value = colon_index != -1 ? msg_recieved->substring(colon_index + 1) : "";



  if (action == PING_ACTION) {
    int time = value.toInt();
    send_operation_and_value(&action, "time", &time, VALUE_LONG);
    return;
  }

  if (action == OBSTACLE_DETECTED_ACTION) {
    float distance = value.toFloat();
    send_operation_and_value(&action, "distance", &distance, VALUE_FLOAT);
    return;
  }

  if (action == LINE_LOST_ACTION) {
    send_operation(&action);
    return;
  }

  if (action == LINE_FOUND_ACTION) {
    send_operation(&action);
    return;
  }

  if (action == INIT_LINE_SEARCH_ACTION || action == END_LINE_SEARCH_ACTION) {
    send_operation(&action);
    return;
  }

  if (action == VISIBLE_LINE) {
    float percentage = value.toFloat();
    send_operation_and_value(&action, "value", &percentage, VALUE_FLOAT);
    return;
  }

  if (action == START_LAP_ACTION) {
    Serial.println("alo");
    send_operation(&action);
    return;
  }

  if (action == END_LAP_ACTION) {
    int time = value.toInt();
    send_operation_and_value(&action, "time", &time, VALUE_LONG);
    return;
  }
}
void loop() {
  if (Serial2.available() > 0) {
    String msg = Serial2.readStringUntil('\n');  // Leer los datos recibidos
    execute_operation(&msg);
  }
}
