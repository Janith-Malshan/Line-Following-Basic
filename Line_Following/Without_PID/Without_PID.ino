// Use this,
//Arduino Uno board, Two IR sensors & L298N motor drive



#define IR_SENSOR_RIGHT 11
#define IR_SENSOR_LEFT 12
#define MOTOR_SPEED 180

//Right Motor
int rightMotorPin1=7;
int rightMotorPin2=8;
int enableRightMotor=6;

//Left Motor
int leftMotorPin1=9;
int leftMotorPin2=10;
int enableLeftMotor=5;

void setup() {
 
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(enableRightMotor, OUTPUT);
  
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(enableLeftMotor, OUTPUT);

  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);
 
  MotorSpeed(0,0);   
}


void loop() {

  int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
  int leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);


  if (rightIRSensorValue == LOW && leftIRSensorValue == LOW)
  {
    MotorSpeed(MOTOR_SPEED, MOTOR_SPEED);
  }
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW )
  {
      MotorSpeed(-MOTOR_SPEED, MOTOR_SPEED); 
  }
  else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH )
  {
      MotorSpeed(MOTOR_SPEED, -MOTOR_SPEED); 
  } 
  else 
  {
    MotorSpeed(0, 0);
  }
  
}


void MotorSpeed(int rightMotorSpeed, int leftMotorSpeed) {

//Right Motor
  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,HIGH);    
  }
  else if (rightMotorSpeed > 0)
  {
    digitalWrite(rightMotorPin1,HIGH);
    digitalWrite(rightMotorPin2,LOW);      
  }
  else
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,LOW);      
  }


//Left Motor
  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,HIGH);    
  }
  else if (leftMotorSpeed > 0)
  {
    digitalWrite(leftMotorPin1,HIGH);
    digitalWrite(leftMotorPin2,LOW);      
  }
  else 
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,LOW);      
  }


  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));    

}
