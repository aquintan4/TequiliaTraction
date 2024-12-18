#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include "FastLED.h"

// Msg consts
#define START_LAP_ACTION "START_LAP"
#define END_LAP_ACTION "END_LAP"
#define OBSTACLE_DETECTED_ACTION "OBSTACLE_DETECTED"
#define LINE_LOST_ACTION "LINE_LOST"
#define PING_ACTION "PING"
#define INIT_LINE_SEARCH_ACTION "INIT_LINE_SEARCH"
#define END_LINE_SEARCH_ACTION "STOP_LINE_SEARCH"
#define LINE_FOUND_ACTION "LINE_FOUND"
#define VISIBLE_LINE "VISIBLE_LINE"


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
#define DTECTION_THRESHOLD 600
#define BASE_SPEED 150

#define DELAY_FOLLOW_LINE 50
#define DELAY_ULTRASOUND 200
#define DELAY_PING 4000


long start_lap;
long num_reed_sensors;
long num_lost_line;
bool seen_obstacle;

// SemaphoreHandle_t xMutex = xSemaphoreCreateMutex();

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

void task_follow_line(void *pvParameters __attribute__((unused))) {
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

  Serial.println("#" + String(START_LAP_ACTION) + "_");

  start_lap = millis();
  bool line_lost = false;

  float correction = 0;
  float error = 0;
  float last_err = 0;

  float dError = 0;


  float Kp = 53.00;  // Ajusta el valor de Kp según sea necesario
  float Kd = 49.0;  // Ajusta el valor de Kd según sea necesario

  for (;;) {
    // Leer sensores
    if (!seen_obstacle){
      num_reed_sensors ++;
      bool sensor_left = analogRead(PIN_LEFT) >= DTECTION_THRESHOLD;
      bool sensor_mid = analogRead(PIN_MIDDLE) >= DTECTION_THRESHOLD;
      bool sensor_right = analogRead(PIN_RIGHT) >= DTECTION_THRESHOLD;

      if (sensor_left || sensor_mid || sensor_right){
        FastLED.showColor(Color(0, 255, 0));
        if (line_lost){
          line_lost = false;
          Serial.println("#" + String(END_LINE_SEARCH_ACTION) + "_");
          Serial.println("#" + String(LINE_FOUND_ACTION) + "_");
        }
      } else{
        FastLED.showColor(Color(255, 0, 0));
        num_lost_line ++;
        if (!line_lost){
          Serial.println("#" + String(LINE_LOST_ACTION) + "_");
          Serial.println("#" + String(INIT_LINE_SEARCH_ACTION) + "_");
          line_lost = true;
        }
      }

      // Calcular el error
      if (sensor_left && !sensor_right) error = -1.0;  // Línea a la izquierda
      else if (sensor_right && !sensor_left) error = 1.0;  // Línea a la derecha
      else if (!sensor_left && !sensor_right && sensor_mid) error = 0.0;  // Línea al centro
      else {
        // Si pierde la línea, aplicar una corrección para intentar recuperarla
        error = (last_err <= 0) ? -2.7 : 2.7; // Gira hacia la izquierda o la derecha
      }

      // Calcular la tasa de cambio del error (derivada)
      dError = error - last_err;

      // Aplicar la fórmula PID (proporcional + derivativo)
      correction = Kp * error + Kd * dError;

      // Almacenar el error anterior para usarlo en el siguiente ciclo
      last_err = error;

      // Calcular velocidades
      float speed_left = BASE_SPEED + correction;
      float speed_right = BASE_SPEED - correction;

      // Redondear y limitar las velocidades para evitar valores fuera de rango
      int speed_left_int = constrain(round(speed_left), 0, 255);
      int speed_right_int = constrain(round(speed_right), 0, 255);  // Corregido para permitir velocidad negativa

      // Aplicar velocidades
      change_vel_motors(speed_left_int, speed_right_int);
    }
    // Esperar el siguiente ciclo
    xTaskDelayUntil(&xLastWakeTime, (DELAY_FOLLOW_LINE / portTICK_PERIOD_MS));
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
  bool first_seen = false;
  seen_obstacle = false;

  for (;;)
  {
    long t; //timepo que demora en llegar el eco
    long d; //distancia en centimetros
  
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);          //Enviamos un pulso de 10us
    digitalWrite(TRIG_PIN, LOW);
    
    t = pulseIn(ECHO_PIN, HIGH); //obtenemos el ancho del pulso
    d = t/59;             //escalamos el tiempo a una distancia en cm
    
    if (d <= 15 && !first_seen){
      change_vel_motors(0, 0);
      seen_obstacle = true;
      if (d <= 8 && !first_seen){
        Serial.println("#" + String(OBSTACLE_DETECTED_ACTION) + ":" + String(d) + "_");
        Serial.println("#" + String(END_LAP_ACTION) + ":" + String(millis() - start_lap) + "_");
        Serial.println("#" + String(VISIBLE_LINE) + ":" + String(((float)(num_reed_sensors - num_lost_line)/(num_reed_sensors) * 100)) + "_");
        first_seen = true;
      }
    }
    xTaskDelayUntil( &xLastWakeTime, ( DELAY_ULTRASOUND / portTICK_PERIOD_MS ));
  }
}

void task_ping( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
  FastLED.setBrightness(20);

  for (;;)
  {
    if (!seen_obstacle){
      FastLED.showColor(Color(0, 0, 255));
      Serial.println("#" + String(PING_ACTION) + ":" + String(millis() - start_lap) + "_");
      xTaskDelayUntil( &xLastWakeTime, ( DELAY_PING / portTICK_PERIOD_MS ));
    }
  }
}
