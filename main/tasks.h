#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include "FastLED.h"

#define PIN_RBGLED 4
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];


#define PIN_LEFT   A2
#define PIN_MIDDLE A1
#define PIN_RIGHT  A0
// Enable/Disable motor control.
//  HIGH: motor control enabled
//  LOW: motor control disabled
#define PIN_Motor_STBY 3

// Group A Motors (Right Side)
// PIN_Motor_AIN_1: Digital output. HIGH: Forward, LOW: Backward
#define PIN_Motor_AIN_1 7
// PIN_Motor_PWMA: Analog output [0-255]. It provides speed.
#define PIN_Motor_PWMA 5

// Group B Motors (Left Side)
// PIN_Motor_BIN_1: Digital output. HIGH: Forward, LOW: Backward
#define PIN_Motor_BIN_1 8
// PIN_Motor_PWMB: Analog output [0-255]. It provides speed.
#define PIN_Motor_PWMB 6

#define TRIG_PIN 13  
#define ECHO_PIN 12  



#define VEL_MOTOR 150

#define DELAY_FOLLOW_LINE 50
#define DELAY_ULTRASOUND 200
#define DELAY_PING 4000

long start;

enum State {
  ALL,
  MID_RIGHT,
  RIGHT,
  MID,
  LEFT,
  MID_LEFT,
  NONE
};

uint32_t Color(uint8_t r, uint8_t g, uint8_t b)
{
  return (((uint32_t)r << 16) | ((uint32_t)g << 8) | b);
}


State get_state_motors(bool left, bool middle, bool right){

  if (left && middle && right){
    return ALL;
  } else if (!left && middle && right){
    return MID_RIGHT;
  } else if (!left && !middle && right){
    return RIGHT;
  } else if (!left && middle && !right){
    return MID;
  } else if (left && !middle && !right){
    return LEFT;
  } else if (left && middle && !right){
    return MID_LEFT;
  } else if (!left && !middle && !right){
    return NONE;
  }
}

void change_vel_motors(int left, int right){
  if (left < 0){
    digitalWrite(PIN_Motor_BIN_1, LOW);
    analogWrite(PIN_Motor_PWMB, -left);
  } else{
    digitalWrite(PIN_Motor_BIN_1, HIGH);
    analogWrite(PIN_Motor_PWMB, left);
  }
  if (right < 0){
    digitalWrite(PIN_Motor_AIN_1, LOW);
    analogWrite(PIN_Motor_PWMA, -right);
  } else{
    digitalWrite(PIN_Motor_AIN_1, HIGH);
    analogWrite(PIN_Motor_PWMA, right);
  }
}

void task_follow_line( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  pinMode(PIN_Motor_STBY, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);

  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
  FastLED.setBrightness(20);

  digitalWrite(PIN_Motor_STBY, HIGH);

  start = millis();

  int dir;

  State current_state = NONE;

  for (;;)
  {
    FastLED.showColor(Color(255, 0, 0));
    bool sensor_left = analogRead(PIN_LEFT) >= 100;;
    bool sensor_mid = analogRead(PIN_MIDDLE) >= 100;
    bool sensor_right = analogRead(PIN_RIGHT) >= 100;

    if (sensor_left && dir != -1){
      dir = -1;
    }
    if (sensor_right && dir != 1){
      dir = 1;
    }
    
    current_state = get_state_motors(sensor_left, sensor_mid, sensor_right);
    switch (current_state) {
      case ALL:
        change_vel_motors(0, 0);
        break;
      case MID_RIGHT:
        change_vel_motors(VEL_MOTOR * 0.9, VEL_MOTOR);
        break;
      case RIGHT:
        change_vel_motors(VEL_MOTOR, 0);
        break;
      case MID:
        change_vel_motors(VEL_MOTOR, VEL_MOTOR);
        break;
      case LEFT:
        change_vel_motors(0, VEL_MOTOR);
        break;
      case MID_LEFT:
        change_vel_motors(VEL_MOTOR, VEL_MOTOR * 0.9);
        break;
      case NONE:
        change_vel_motors(VEL_MOTOR * dir * 0.5, -VEL_MOTOR * dir * 0.5+);
        break;
    }
    xTaskDelayUntil( &xLastWakeTime, ( DELAY_FOLLOW_LINE / portTICK_PERIOD_MS ));
  }
}

void task_obstacle( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  pinMode(PIN_Motor_STBY, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT); //pin como salida
  pinMode(ECHO_PIN, INPUT);  //pin como entrada
  digitalWrite(TRIG_PIN, LOW);//Inicializamos el pin con 0

  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
  FastLED.setBrightness(20);

  for (;;)
  {
    FastLED.showColor(Color(0, 255, 0));
    long t; //timepo que demora en llegar el eco
    long d; //distancia en centimetros
  
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);          //Enviamos un pulso de 10us
    digitalWrite(TRIG_PIN, LOW);
    
    t = pulseIn(ECHO_PIN, HIGH); //obtenemos el ancho del pulso
    d = t/59;             //escalamos el tiempo a una distancia en cm
    
    if (d <= 8){
      digitalWrite(PIN_Motor_STBY, LOW);
    } else{
      digitalWrite(PIN_Motor_STBY, HIGH);
    }
    xTaskDelayUntil( &xLastWakeTime, ( DELAY_ULTRASOUND / portTICK_PERIOD_MS ));
  }
}
void task_ping( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  String ping;
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
  FastLED.setBrightness(20);

  for (;;)
  {
    ping = "#PING:" + String(millis() - start) + "_";
    FastLED.showColor(Color(0, 0, 255));
    
    Serial.print(ping.c_str());
    xTaskDelayUntil( &xLastWakeTime, ( DELAY_PING / portTICK_PERIOD_MS ));
  }
}

