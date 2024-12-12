
// #include <WiFi.h> //Wifi library
// #include "secret.h"

// //SSID NAME


// void setup() {
//   Serial.begin(115200);
//   delay(10);
//   Serial.print(F("Connecting to network: "));
//   Serial.println(ssid);
//   WiFi.disconnect(true); 
//   WiFi.mode(WIFI_STA);
//   // WiFi.begin(ssid, WPA2_AUTH_PEAP, EAP_IDENTITY, EAP_USERNAME, EAP_PASSWORD); 

//   WiFi.begin(ssid, pwd); 

//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(F("."));
//   }
//   Serial.println("");
//   Serial.println(F("WiFi is connected!"));
//   Serial.println(F("IP address set: "));
//   Serial.println(WiFi.localIP()); //print LAN IP
// }

// void loop() {
//   yield();
// }


#include <WiFi.h> // Wifi library
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

// #include "secret.h"

#define AIO_SERVER "193.147.79.118"
#define AIO_SERVERPORT 21883
#define TOPIC "/SETR/2024/7"

// SSID NAME
const char* ssid = "POCO M3"; // eduroam SSID
const char* pwd = "pepepepe";

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT);
Adafruit_MQTT_Publish pub = Adafruit_MQTT_Publish(&mqtt, TOPIC);

typedef union {
  struct __attribute__((__packed__)) {
    char team_name[16];
    uint16_t id;
    char action[16];
  } msg;
  uint8_t raw[sizeof(msg)];
} packet_t;

packet_t pruebas_msg;

void setup() {
  int ret;

  Serial.begin(115200);
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

  // Inicialización de los datos en pruebas_msg
  strncpy(pruebas_msg.msg.team_name, "TequilaTraction", sizeof(pruebas_msg.msg.team_name) - 1);
  pruebas_msg.msg.team_name[sizeof(pruebas_msg.msg.team_name) - 1] = '\0';  // Asegurarse de que está bien terminado en null
  pruebas_msg.msg.id = 7;
  strncpy(pruebas_msg.msg.action, "START_LAP", sizeof(pruebas_msg.msg.action) - 1);
  pruebas_msg.msg.action[sizeof(pruebas_msg.msg.action) - 1] = '\0';  // Asegurarse de que está bien terminado en null
}

void loop() {

  if (!pub.publish(pruebas_msg.raw, sizeof(pruebas_msg))) {
    Serial.println("Error al publicar");
  } else {
    Serial.println("Publicado");
    delay(500);
  }

  delay(2000);
}
