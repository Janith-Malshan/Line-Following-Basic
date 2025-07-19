#include <Wire.h>

#define THRESHOLD 900

#define MAX_SPEED 150
#define turn_speed 150

int MotorBasespeed = 110;

// IR sensor pins
#define IR1 A0
#define IR2 A1
#define IR3 A2
#define IR4 A3
#define IR5 A4
#define IR6 A5

#define IR_RIGHT_SENSOR_PIN 4  // Left turn IR sensor 
#define IR_LEFT_SENSOR_PIN 2 // Right turn IR sensor 

bool rightTurnDetected = false;    // Flag for sharp left turn detection 
bool leftTurnDetected = false;   // Flag for sharp right turn detection 

int IR_VAL[6] = {0, 0, 0, 0, 0, 0};
int IR_weights[6] = {-20, -10, -5, 5, 10, 20};
int IR_PINS[6] = {A5, A4, A3, A2, A1, A0};

// Motor driver pins
#define ENB 10    // Right motor speed (Motor B)
#define IN3 8     // Right motor IN1
#define IN4 9     // Right motor IN2

#define ENA 13    // Left motor speed (Motor A)
#define IN1 11    // Left motor IN1
#define IN2 12    // Left motor IN2

#define STBY 7    // Standby pin

int RMOTORSpeed = 0;
int LMOTORSpeed = 0;
int speedAdjust = 0;

// PID variables
float P, I, D;
float error = 0;
float previousError = 0;
float Kp = 1.25;
float Kd = 1.25;
float Ki = 0;

void PID_control();
void read_IR();
void set_speed();
void set_forward();
void set_backward();
void linefollowing(int baseSpeed);

int clickCount = 0;
bool lastButtonState = HIGH;



void setup() {
  // Set motor driver pins as output
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Enable motor driver
  digitalWrite(STBY, HIGH);

  pinMode(IR_RIGHT_SENSOR_PIN, INPUT);
  pinMode(IR_LEFT_SENSOR_PIN, INPUT);

}






void loop() {
  linefollowing(70);
}





void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}





void sharpLeftTurn() {
  // Stop left motor and spin right motor in reverse to make a sharp right turn 
  set_speed(LMOTORSpeed);   // Stop left motor 
  set_speed(-LMOTORSpeed);  // Spin right motor backward 
  delay(510);             // Adjust time for desired sharp turn 
  stopMotors();           // Stop motors after the turn 
}




// ---------------- PID CONTROL ----------------
void PID_control() {
  error = 0;
  for (int i = 0; i < 6; i++) {
    error += IR_weights[i] * IR_VAL[i];
  }

  P = error;
  I += error;
  D = error - previousError;
  previousError = error;

  speedAdjust = (Kp * P + Ki * I + Kd * D);

  RMOTORSpeed = MotorBasespeed - speedAdjust;
  LMOTORSpeed = MotorBasespeed + speedAdjust;

  RMOTORSpeed = constrain(RMOTORSpeed, 0, MAX_SPEED);
  LMOTORSpeed = constrain(LMOTORSpeed, 0, MAX_SPEED);
}

// ---------------- IR SENSOR READ ----------------
void read_IR() {
  for (int i = 0; i < 6; i++) {
    int analogValue = analogRead(IR_PINS[i]);
    IR_VAL[i] = (analogValue > THRESHOLD) ? 1 : 0;
  }
}

// ---------------- SET MOTOR SPEEDS ----------------
void set_speed() {
  analogWrite(ENA, LMOTORSpeed);  // Left Motor
  analogWrite(ENB, RMOTORSpeed);  // Right Motor
}

// ---------------- SET FORWARD DIRECTION ----------------
void set_forward() {
  // Left motor forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Right motor forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// ---------------- SET BACKWARD DIRECTION (optional) ----------------
void set_backward() {
  // Left motor backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  // Right motor backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// ---------------- MAIN LINE FOLLOWING FUNCTION ----------------
void linefollowing(int baseSpeed) {
  MotorBasespeed = baseSpeed;
  set_forward();
  read_IR();
  PID_control();
  set_speed();
}
