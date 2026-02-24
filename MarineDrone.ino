/*
 * PROJECT: AUTONOMOUS MARINE DRONE
 * DESCRIPTION: Autonomous navigation using GPS, Sonar for obstacle avoidance, 
 * [span_0](start_span)[span_1](start_span)and Servo-controlled anchor system.[span_0](end_span)[span_1](end_span)
 * * HARDWARE MAPPING:
 * - [span_2](start_span)ESP32 WROOM Dev[span_2](end_span)
 * - [span_3](start_span)Neo 6M GPS (Serial2: RX16, TX17)[span_3](end_span)
 * - [span_4](start_span)L-298 Motor Driver (Pins 14, 27, 26, 25, 33, 32)[span_4](end_span)
 * - [span_5](start_span)HCSR04 Sonar (Trig 5, Echo 13)[span_5](end_span)
 * - [span_6](start_span)SG90 Micro Servo (Pin 4)[span_6](end_span)
 */

#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <ESP32Servo.h>

// --- PIN DEFINITIONS ---
[span_7](start_span)[span_8](start_span)// Propulsion[span_7](end_span)[span_8](end_span)
const int ENA = 14; 
const int IN1 = 27; 
const int IN2 = 26; 
const int IN3 = 25; 
const int IN4 = 33; 
const int ENB = 32; 

[span_9](start_span)// Electronics & Sensors[span_9](end_span)
const int TRIG_PIN = 5;
const int ECHO_PIN = 13;
const int SERVO_PIN = 4;

// --- NAVIGATION CONFIGURATION ---
// Replace with your desired destination coordinates
double targetLat = 18.5204; 
double targetLon = 73.8567; 
const int DISTANCE_THRESHOLD = 5; // Stop within 5 meters

// PWM Settings for ESP32 v3.0.0
const int PWM_FREQ = 20000; 
const int PWM_RES  = 8;     

// --- OBJECTS ---
TinyGPSPlus gps;
HardwareSerial SerialGPS(2); 
Servo anchorServo;

void setup() {
  Serial.begin(115200);
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17); [span_10](start_span)//[span_10](end_span)
  
  [span_11](start_span)// Initialize Motor Pins[span_11](end_span)
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  
  [span_12](start_span)// Initialize Sonar[span_12](end_span)
  pinMode(TRIG_PIN, OUTPUT); 
  pinMode(ECHO_PIN, INPUT);

  // Configure Motor Speed Control (New ESP32 API)
  ledcAttach(ENA, PWM_FREQ, PWM_RES);
  ledcAttach(ENB, PWM_FREQ, PWM_RES);
  
  [span_13](start_span)// Initialize Anchor[span_13](end_span)
  anchorServo.attach(SERVO_PIN);
  anchorServo.write(0); [span_14](start_span)// Position: Up[span_14](end_span)
  delay(1000);
  anchorServo.detach(); [span_15](start_span)// Prevent overheating risk[span_15](end_span)

  Serial.println("SYSTEM INITIALIZED: Awaiting GPS Fix...");
}

void loop() {
  [span_16](start_span)// Feed GPS data[span_16](end_span)
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }

  [span_17](start_span)// 1. OBSTACLE AVOIDANCE (Priority)[span_17](end_span)
  float dist = readSonar();
  [span_18](start_span)if (dist > 0 && dist < 100) { // Object within 100cm[span_18](end_span)
    Serial.println("OBSTACLE DETECTED! Executing Evasive Maneuver.");
    executeEvasiveAction();
    return; 
  }

  [span_19](start_span)// 2. AUTONOMOUS NAVIGATION[span_19](end_span)
  if (gps.location.isValid()) {
    double distanceToTarget = TinyGPSPlus::distanceBetween(
      gps.location.lat(), gps.location.lng(), targetLat, targetLon);

    if (distanceToTarget > DISTANCE_THRESHOLD) {
      double courseToTarget = TinyGPSPlus::courseTo(
        gps.location.lat(), gps.location.lng(), targetLat, targetLon);
      
      smartNavigate(courseToTarget, gps.course.deg());
    } else {
      Serial.println("MISSION COMPLETE: Destination Reached.");
      stopMotors();
      deployAnchor(); [span_20](start_span)//[span_20](end_span)
      while(1); // End mission
    }
  }
}

// --- CORE FUNCTIONS ---

float readSonar() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  return (duration == 0) ? -1 : (duration * 0.034 / 2);
}

void smartNavigate(double targetHeading, double currentHeading) {
  double error = targetHeading - currentHeading;
  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  if (abs(error) < 20) { 
    moveForward(200); 
  } else if (error > 0) {
    turnRight(180);
  } else {
    turnLeft(180);
  }
}

void deployAnchor() {
  anchorServo.attach(SERVO_PIN);
  [span_21](start_span)// Slow deployment to mitigate mechanical jamming[span_21](end_span)
  for(int pos = 0; pos <= 160; pos += 2) { 
    anchorServo.write(pos); 
    delay(25); 
  }
  anchorServo.detach(); [span_22](start_span)// Mitigate overheating risk[span_22](end_span)
}

void executeEvasiveAction() {
  stopMotors();
  delay(500);
  moveReverse(180); [span_23](start_span)//[span_23](end_span)
  delay(1500);
  turnRight(200);
  delay(1000);
  stopMotors();
}

void moveForward(int s) { 
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); ledcWrite(ENA, s);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); ledcWrite(ENB, s);
}

void moveReverse(int s) { 
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); ledcWrite(ENA, s);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); ledcWrite(ENB, s);
}

void turnLeft(int s) { 
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); ledcWrite(ENA, s);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); ledcWrite(ENB, s);
}

void turnRight(int s) { 
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); ledcWrite(ENA, s);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); ledcWrite(ENB, s);
}

void stopMotors() { ledcWrite(ENA, 0); ledcWrite(ENB, 0); }
