#include "tasks.h"



void setup() {
  
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);


  while (!Serial) {
    ; // wait for serial port to connect.
  }

  // wait for a msg sent by the ESP32 to begin the program
  for (;;){
    if (Serial.available() > 0){
      String msg = Serial.readStringUntil('_');;
      if (msg == "#commReady"){
        Serial.print("#ACK_"); // if the msg is correct it send another msg to the ESP
        }
        break;
    }
  }
  // task for the follow line part
  xTaskCreate(
    task_follow_line
    ,  "Follow_Line"
    ,  200 
    ,  NULL
    ,  3  
    ,  NULL );
  // task for the obstacle part
  xTaskCreate(
  task_obstacle
  ,  "Obstacle"
  ,  200 
  ,  NULL
  ,  2 
  ,  NULL );
  // task for the ping part
  xTaskCreate(
  task_ping
  ,  "Ping"  
  ,  100  
  ,  NULL
  ,  1
  ,  NULL );
}

void loop()
{
  // Empty. Things are done in Tasks.
}
