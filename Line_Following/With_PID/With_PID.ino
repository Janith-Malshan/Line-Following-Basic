//Use these,
//Arduino Mega board, QTR8A sensor panel, L298N motor driver




const int motorLeftPin1 = 5;
const int motorLeftPin2 = 6;
const int motorRightPin1 = 9;
const int motorRightPin2 = 10;
const int sensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7}; ;// Define QTR-8A sensor pins and motor pins

// PID constants
float Kp = 0.5;  
float Ki = 0.0;   
float Kd = 1.0;   

// PID variables
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;
float correction = 0;

int baseSpeed = 150;  // Base speed for motors
int maxSpeed = 255;   // Maximum motor speed

void setup() {
  for (int i = 0; i < 8; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  pinMode(motorLeftPin1, OUTPUT);
  pinMode(motorLeftPin2, OUTPUT);
  pinMode(motorRightPin1, OUTPUT);
  pinMode(motorRightPin2, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  int position = readSensors();  // Get the position from sensors
  error = position - 3500;       // Target position is centered at 3500

  // Calculate PID terms
  integral += error;
  derivative = error - lastError;
  correction = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  int motorSpeedLeft = constrain(baseSpeed + correction, 0, maxSpeed);
  int motorSpeedRight = constrain(baseSpeed - correction, 0, maxSpeed);

  // Set motor speeds
  analogWrite(motorLeftPin1, motorSpeedLeft);
  analogWrite(motorRightPin1, motorSpeedRight);
  analogWrite(motorLeftPin2, 0);  // Only forward
  analogWrite(motorRightPin2, 0); // Only forward

  delay(10);
}

// Function to read sensors and calculate position
int readSensors() {
  long totalValue = 0;
  int sum = 0;

  for (int i = 0; i < 8; i++) {
    int sensorValue = analogRead(sensorPins[i]);
    totalValue += sensorValue * (i * 1000);  // Weight each sensor
    sum += sensorValue;
  }

  return sum > 0 ? totalValue / sum : 3500;  // Return position or center if no line
}