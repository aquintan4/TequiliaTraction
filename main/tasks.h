#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include "FastLED.h"


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

//========= LED CONSTANTS ==========

#define PIN_RBGLED 4
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

//======= INFRARED CONSTANTS =======

#define PIN_LEFT   A2
#define PIN_MIDDLE A1
#define PIN_RIGHT  A0

#define DTECTION_THRESHOLD 600

//========= MOTOR CONSTANTS ========

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

//====== ULTRASOUND CONSTANTS ======

#define TRIG_PIN 13  
#define ECHO_PIN 12
#define MICRODELAY_ULTRASOUND 10

#define THRESHOLD_REDUCE_SPEED 25
#define THRESHOLD_STOP 7


//======== SPEED CONSTANTS =========

#define NORMAL_SPEED 175
#define REDUCED_SPEED 50


//======== ERROR CONSTANTS =========

#define ERROR_LINE 1.0
#define ERROR_NO_LINE 3.1


//=========== DELAYS ===============

#define DELAY_FOLLOW_LINE 20
#define DELAY_ULTRASOUND 100
#define DELAY_PING 4000



// ======= GLOBAL VARIABLES ========
int base_speed = NORMAL_SPEED;        // Base speed for the motors
long start_lap;              // Timestamp for the start of the lap
long num_reed_sensors;       // Counter for line sensor readings
long num_lost_line;          // Counter for line lost events
bool seen_obstacle;          // Flag to indicate if an obstacle is detected


uint32_t Color(uint8_t r, uint8_t g, uint8_t b)
{
  return (((uint32_t)r << 16) | ((uint32_t)g << 8) | b);
}

void change_vel_motors(int left, int right){

  // The parameters are the speed of each side


  // Left motor configuration
  if (left < 0) {
    digitalWrite(PIN_Motor_BIN_1, LOW);          // Reverse direction
    analogWrite(PIN_Motor_PWMB, -left);         // Set the speed
  } else {
    digitalWrite(PIN_Motor_BIN_1, HIGH);         // Forward direction
    analogWrite(PIN_Motor_PWMB, left);          // Set the speed
  }

  // Right motor configuration
  if (right < 0) {
    digitalWrite(PIN_Motor_AIN_1, LOW);          // Reverse direction
    analogWrite(PIN_Motor_PWMA, -right);        // Set the speed
  } else {
    digitalWrite(PIN_Motor_AIN_1, HIGH);         // Forward direction
    analogWrite(PIN_Motor_PWMA, right);         // Set the speed
  }
}


void task_follow_line(void *pvParameters __attribute__((unused))) {
  // Initialize timing for task scheduling
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  // Configure motor pins as outputs
  pinMode(PIN_Motor_STBY, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);

  // Initialize LEDs with FastLED library
  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
  FastLED.setBrightness(20); // Set LED brightness

  // Enable the motors
  digitalWrite(PIN_Motor_STBY, HIGH);

  Serial.println("#" + String(START_LAP_ACTION) + "_");

  start_lap = millis();  // Store start time
  bool line_lost = false; // Track if the line is lost

  // Variables for PID control
  float correction = 0; 
  float error = 0;
  float last_err = 0;
  float dError = 0;

  // PID tuning parameters
  float Kp = 57.00; // Proportional gain
  float Kd = 54.5;  // Derivative gain

  for (;;) {
    // Read line sensors only if no obstacle is detected
    if (!seen_obstacle){
      num_reed_sensors ++;
      bool sensor_left = analogRead(PIN_LEFT) >= DTECTION_THRESHOLD;
      bool sensor_mid = analogRead(PIN_MIDDLE) >= DTECTION_THRESHOLD;
      bool sensor_right = analogRead(PIN_RIGHT) >= DTECTION_THRESHOLD;

      // Handle line detection
      if (sensor_left || sensor_mid || sensor_right) {
        FastLED.showColor(Color(0, 255, 0)); // Green LED for line detected
        if (line_lost) {
          line_lost = false;
          Serial.println("#" + String(END_LINE_SEARCH_ACTION) + "_");
          Serial.println("#" + String(LINE_FOUND_ACTION) + "_");
        }
      } else {
        // If no line is detected
        FastLED.showColor(Color(255, 0, 0)); // Red LED for lost line
        num_lost_line++; // Increment lost line count
        if (!line_lost) {
          Serial.println("#" + String(LINE_LOST_ACTION) + "_");
          Serial.println("#" + String(INIT_LINE_SEARCH_ACTION) + "_");
          line_lost = true;
        }
      }

      // Calculate error for PID control
      if (sensor_left && !sensor_right) error = -1 * ERROR_LINE;  // Line is to the left
      else if (sensor_right && !sensor_left) error = ERROR_LINE;  // Line is to the right
      else if (!sensor_left && !sensor_right && sensor_mid) error = 0.0;  // Line is centered
      else {
        // If line is lost, apply correction based on last error
        error = (last_err <= 0) ? -1 * ERROR_NO_LINE : ERROR_NO_LINE; // Turn left or right
      }

      // Calculate the rate of change of error (derivative)
      dError = error - last_err;

      // Apply PID formula: proportional + derivative
      correction = Kp * error + Kd * dError;

      // Store the current error as last error for the next iteration
      last_err = error;

      // Calculate motor speeds with correction
      float speed_left = base_speed + correction;
      float speed_right = base_speed - correction;

      // Round and constrain speeds to valid range [0-255]
      int speed_left_int = constrain(round(speed_left), 0, 255);
      int speed_right_int = constrain(round(speed_right), 0, 255);

      // Apply motor speeds
      change_vel_motors(speed_left_int, speed_right_int);
    }
    xTaskDelayUntil(&xLastWakeTime, (DELAY_FOLLOW_LINE / portTICK_PERIOD_MS));
  }
}



void task_obstacle( void *pvParameters __attribute__((unused)) )
{
  // Initialize timing for task scheduling
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  // Configure motor standby and ultrasonic sensor pins
  pinMode(PIN_Motor_STBY, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT); // Trigger pin set as output
  pinMode(ECHO_PIN, INPUT);  // Echo pin set as input
  digitalWrite(TRIG_PIN, LOW); // Initialize trigger pin to LOW

  // Initialize LEDs with FastLED library
  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
  FastLED.setBrightness(20); // Set LED brightness

  seen_obstacle = false;     // Global flag to indicate obstacle presence

  for (;;)
  {
    long t; // Time taken for the echo pulse
    long d; // Calculated distance in centimeters
  
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(MICRODELAY_ULTRASOUND);          //Enviamos un pulso de 10us
    digitalWrite(TRIG_PIN, LOW);
    
    // Measure the time of the echo pulse
    t = pulseIn(ECHO_PIN, HIGH);

    // Convert the time to distance in centimeters
    d = t/59;

    // Check if the obstacle is within a critical distance
    if (d <= THRESHOLD_REDUCE_SPEED){
      base_speed = NORMAL_SPEED; // Reduce base speed when an obstacle is nearby
      if (d <= THRESHOLD_STOP){
        change_vel_motors(0,0); // Stop motors if the obstacle is very close
        Serial.println("#" + String(OBSTACLE_DETECTED_ACTION) + ":" + String(d) + "_");
        Serial.println("#" + String(END_LAP_ACTION) + ":" + String(millis() - start_lap) + "_");
        Serial.println("#" + String(VISIBLE_LINE) + ":" + String(((float)(num_reed_sensors - num_lost_line)/(num_reed_sensors) * 100)) + "_");
        seen_obstacle = true;// Update global flag
      }
    }
    xTaskDelayUntil( &xLastWakeTime, ( DELAY_ULTRASOUND / portTICK_PERIOD_MS ));
  }
}

void task_ping( void *pvParameters __attribute__((unused)) ){
// Initialize timing for task scheduling
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  // Initialize LEDs with FastLED library
  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
  FastLED.setBrightness(20); // Set LED brightness

  for (;;) {
    // Only send a ping if no obstacle has been detected
    if (!seen_obstacle) {
      FastLED.showColor(Color(0, 0, 255)); // Show blue LED for a ping
      Serial.println("#" + String(PING_ACTION) + ":" + String(millis() - start_lap) + "_"); // Log a ping message
      xTaskDelayUntil(&xLastWakeTime, (DELAY_PING / portTICK_PERIOD_MS));
    }
  }
}
