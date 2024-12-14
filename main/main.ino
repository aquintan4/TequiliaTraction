#include "tasks.h"



void setup() {
  
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);


  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

  for (;;){
    if (Serial.available() > 0){
      String msg = Serial.readStringUntil('_');;
      if (msg == "#commReady"){
        Serial.print("#ACK_");
        break;
      }
    }
  }




  // Now set up two tasks to run independently.
  xTaskCreate(
    task_follow_line
    ,  "Follow_Line"   // A name just for humans
    ,  100  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
    xTaskCreate(
    task_obstacle
    ,  "Obstacle"   // A name just for humans
    ,  100  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
    xTaskCreate(
    task_ping
    ,  "Ping"   // A name just for humans
    ,  100  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
}

void loop()
{
  // Empty. Things are done in Tasks.
}
