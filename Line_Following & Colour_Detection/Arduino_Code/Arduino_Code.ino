#include <Wire.h>
#include <Servo.h>

#define THRESHOLD 900

#define MAX_SPEED 150
#define turn_speed 150

//#define BUTTON_PIN 2

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
bool red_detected = 0;

char xColor = '-';
char yColor = '-';
char detectedColor = '-';
String inputString = "" ;

bool startCommandReceived = false;
bool firstColorDone = false;
bool secondColorDone = false;
bool bothColorsDone = false;

Servo myServo;         //up down
Servo myServoGripper;  //gripper


void setup() {

  Serial.begin(9600);
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

 // pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.println("Setup complete");

  myServo.attach(22);  // 0deg -lower, 85deg -upper
  myServoGripper.attach(24);

  delay(5);

  myServo.write(120);
  myServoGripper.write(10);

  Start();

}





void delayWithoutBlocking(unsigned long waitTime) {
  unsigned long start = millis();
  while (millis() - start < waitTime) {
    updateDetectedColor();  // Continue reading from serial during delay
  }
}





void updateDetectedColor() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    Serial.print(inChar);  // ? Debug print to monitor input live
    if (inChar == '\n') {
      processInput(inputString);
      inputString = "";
    } else {
      inputString += inChar;
    }
  }
}






void loop() {

  updateDetectedColor();

  // Wait for start command
  if (!startCommandReceived) return;

  
  // Step 1: Run first color routine (xColor)
  if (!firstColorDone && (xColor == 'R' || xColor == 'G' || xColor == 'B')) {
    Serial.println("Starting first color routine...");
    //runColorRoutine(xColor);
    selection_1(xColor);
    firstColorDone = true;
    Serial.println("First color routine completed.");
    //Game_Start();  // Optional transition motion
  }

  // Step 2: After first routine is done, run second color routine (yColor)
  if (firstColorDone && !secondColorDone && (yColor == 'R' || yColor == 'G' || yColor == 'B')) {
    Serial.println("Starting second color routine...");
    //runColorRoutine(yColor);
    selection_2(yColor);
    secondColorDone = true;
    Serial.println("Second color routine completed.");

    // Step 3: Reset everything
    startCommandReceived = false;
    firstColorDone = false;
    secondColorDone = false;
    bothColorsDone = true;
    xColor = '-';
    yColor = '-';
    detectedColor = '-';
    Serial.println("All routines completed. Waiting for new command.");
  }

  if (bothColorsDone) {
    Turning_Left2();
    END();
    Serial.println("All are Done.");
  }

}
// END





void processInput(String input) {
  input.trim();

  if (input.startsWith("X:")) {
    xColor = input.charAt(2);
    Serial.print("X color set to: ");
    Serial.println(xColor);

  } else if (input.startsWith("Y:")) {
    yColor = input.charAt(2);
    Serial.print("Y color set to: ");
    Serial.println(yColor);

  } else if (input.startsWith("D:")) {
    detectedColor = input.charAt(2);
    Serial.print("Detected color from camera: ");
    Serial.println(detectedColor);
    
  } else if (input.startsWith("S:")) {
    if (input.charAt(2) == '1') {
      startCommandReceived = true;
      Serial.println("Start signal received.");
    }
  }
}





// This runs the color function based on input
void runColorRoutine(char color) {
  if (color == 'R') Colour_Detected_Red();
  else if (color == 'G') Colour_Detected_Green();
  else if (color == 'B') Colour_Detected_Blue();
}





void selection_1 (char color) {
  Serial.println("Transition between first and second color...");
  Turning_Left();
  delay(500);
  Turning_Right();
  delay(500);
  //linefollowing(50);
  runColorRoutine(color);   // Start the color routine
}





void selection_2 (char color) {
  Serial.println("Transition between first and second color...");
  Turning_Right2();
  Turning_Left();
  Turning_Right3();
  Pass_Black_Box_2();
  Turning_Left();
  Turning_Right();
  //linefollowing(50);
  runColorRoutine(color);   // Start the color routine
}





void Colour_Detected_Red () {

  int red_detected = 0 ;
  int AllDone = 0 ;

  do {
    linefollowing(55);
    updateDetectedColor();

    if (detectedColor == 'R') {  // pick - 1st position REDnm
      red_detected = 1;

      delayWithoutBlocking(50);
      stopMotors();
      delayWithoutBlocking(500);
      Backward_To_Line();
      delayWithoutBlocking(500);
      myServoGripper.write(90);
      delayWithoutBlocking(500);
      myServo.write(0);
      delayWithoutBlocking(500);
      myServoGripper.write(10);
      delayWithoutBlocking(500);
      myServo.write(120);
      delayWithoutBlocking(500);
      On_To_Line2();
      delayWithoutBlocking(500);
      Turning_180Left();
      delayWithoutBlocking(500);
      Turning_Left();
      delayWithoutBlocking(500);
      Turning_Right();
      delayWithoutBlocking(500);
      Pass_Black_Box();
      delayWithoutBlocking(500);
      Turning_Left3();
      delayWithoutBlocking(500);
      Turning_Right();
      delayWithoutBlocking(300);
      Turning_Left3();
      delayWithoutBlocking(500);
      Turning_Right();

      do {
        updateDetectedColor();
        linefollowing(55);
        updateDetectedColor();

        if (detectedColor == 'R') { // place - 1st position REDnm
          delayWithoutBlocking(50);
          stopMotors();
          delayWithoutBlocking(500);
          Backward_To_Line();
          delayWithoutBlocking(500);
          myServo.write(0);
          delayWithoutBlocking(500);
          myServoGripper.write(90);
          delayWithoutBlocking(500);
          myServo.write(120);
          delayWithoutBlocking(500);
          myServoGripper.write(10);
          delayWithoutBlocking(500);
          On_To_Line2();
          delayWithoutBlocking(500);
          Turning_180Left();
          delayWithoutBlocking(500);
          Turning_Left();
          delayWithoutBlocking(500);
          Turning_Right();
          return;
        }
        else if (detectedColor == 'G' || detectedColor == 'B') { // 1st RED neweinm
          delayWithoutBlocking(50);
          stopMotors();
          delayWithoutBlocking(500);
          On_To_Line();
          Turning_180Left();
          delayWithoutBlocking(500);
          Turning_Left2();
          delayWithoutBlocking(500);

          do {
            updateDetectedColor();
            linefollowing(55);
            updateDetectedColor();

            if (detectedColor == 'R') { // 2nd REDnm
              delayWithoutBlocking(50);
              stopMotors();
              delayWithoutBlocking(500);
              Backward_To_Line();
              delayWithoutBlocking(500);
              myServo.write(0);
              delayWithoutBlocking(500);
              myServoGripper.write(90);
              delayWithoutBlocking(500);
              myServo.write(120);
              delayWithoutBlocking(500);
              myServoGripper.write(10);
              delayWithoutBlocking(500);
              On_To_Line2();
              delayWithoutBlocking(500);
              Turning_180Left();
              delayWithoutBlocking(500);
              Stop_at_Black_Line();
              return;
            }
            else if (detectedColor == 'G' || detectedColor == 'B') { // 2nd RED neweinm
              delayWithoutBlocking(50);
              stopMotors();
              delayWithoutBlocking(500);
              On_To_Line();
              delayWithoutBlocking(500);
              Turning_180Left();
              delayWithoutBlocking(500);
              Turning_Left2();
              delayWithoutBlocking(500);

              do {
                updateDetectedColor();
                linefollowing(55);
                updateDetectedColor();

                if (detectedColor == 'R') { // 3rd REDnm
                  delayWithoutBlocking(50);
                  stopMotors();
                  delayWithoutBlocking(500);
                  Backward_To_Line();
                  delayWithoutBlocking(500);
                  myServo.write(0);
                  delayWithoutBlocking(500);
                  myServoGripper.write(90);
                  delayWithoutBlocking(500);
                  myServo.write(120);
                  delayWithoutBlocking(500);
                  myServoGripper.write(10);
                  delayWithoutBlocking(500);
                  On_To_Line2();
                  delayWithoutBlocking(500);
                  Turning_180Left();
                  delayWithoutBlocking(500);
                  Turning_Right();
                  delayWithoutBlocking(500);
                  Turning_Left();
                  delayWithoutBlocking(500);
                  return;
                }
              } while (1);
            }
          } while (1);
        }
      } while (1);

      AllDone = 1 ;

      
    }



    else if (detectedColor == 'G' || detectedColor == 'B') { // pick - 1st position RED neweinm
      delayWithoutBlocking(50);
      stopMotors();
      delayWithoutBlocking(500);
      On_To_Line();
      delayWithoutBlocking(500);
      Turning_180Left();
      delayWithoutBlocking(500);
      Turning_Left2();
      delayWithoutBlocking(500);

      do {
        updateDetectedColor();
        linefollowing(55);
        updateDetectedColor();

        if (detectedColor == 'R') { // 2nd position REDnm
          updateDetectedColor();
          //Serial.println("Detected color from camera: R");

          //red_detected = 1;
          delayWithoutBlocking(50);
          stopMotors();
          delayWithoutBlocking(500);
          Backward_To_Line();
          delayWithoutBlocking(500);
          myServoGripper.write(90);
          delayWithoutBlocking(500);
          myServo.write(0);
          delayWithoutBlocking(500);
          myServoGripper.write(10);
          delayWithoutBlocking(500);
          myServo.write(120);
          delayWithoutBlocking(500);
          On_To_Line2();
          delayWithoutBlocking(500);
          Turning_180Left();
          delayWithoutBlocking(500);
          Stop_at_Black_Line();
          delayWithoutBlocking(500);
          Pass_Black_Box();
          delayWithoutBlocking(500);
          Turning_Left3();
          delayWithoutBlocking(500);
          Turning_Right();
          delayWithoutBlocking(300);
          Turning_Left3();
          delayWithoutBlocking(500);
          Turning_Right();

          do {
            updateDetectedColor();
            linefollowing(55);
            updateDetectedColor();

            if (detectedColor == 'R') { // place - 1st REDnm
              delayWithoutBlocking(50);
              stopMotors();
              delayWithoutBlocking(500);
              Backward_To_Line();
              delayWithoutBlocking(500);
              myServo.write(0);
              delayWithoutBlocking(500);
              myServoGripper.write(90);
              delayWithoutBlocking(500);
              myServo.write(120);
              delayWithoutBlocking(500);
              myServoGripper.write(10);
              delayWithoutBlocking(500);
              On_To_Line2();
              delayWithoutBlocking(500);
              Turning_180Left();
              delayWithoutBlocking(500);
              Turning_Left();
              delayWithoutBlocking(500);
              Turning_Right();
              return;
            }
            else if (detectedColor == 'G' || detectedColor == 'B') { // 1st RED neweinm
              delayWithoutBlocking(50);
              stopMotors();
              delayWithoutBlocking(500);
              On_To_Line();
              delayWithoutBlocking(500);
              Turning_180Left();
              delayWithoutBlocking(500);
              Turning_Left2();
              delayWithoutBlocking(500);

              do {
                updateDetectedColor();
                linefollowing(55);
                updateDetectedColor();

                if (detectedColor == 'R') { // 2nd place REDnm
                  delayWithoutBlocking(50);
                  stopMotors();
                  delayWithoutBlocking(500);
                  Backward_To_Line();
                  delayWithoutBlocking(500);
                  myServo.write(0);
                  delayWithoutBlocking(500);
                  myServoGripper.write(90);
                  delayWithoutBlocking(500);
                  myServo.write(120);
                  delayWithoutBlocking(500);
                  myServoGripper.write(10);
                  delayWithoutBlocking(500);
                  On_To_Line2();
                  delayWithoutBlocking(500);
                  Turning_180Left();
                  delayWithoutBlocking(500);
                  Stop_at_Black_Line();
                  return;
                }
                else if (detectedColor == 'G' || detectedColor == 'B') { // 2nd place RED neweinm
                  delayWithoutBlocking(50);
                  stopMotors();
                  delayWithoutBlocking(500);
                  On_To_Line();
                  delayWithoutBlocking(500);
                  Turning_180Left();
                  delayWithoutBlocking(500);
                  Turning_Left2();
                  delayWithoutBlocking(500);

                  do {
                    updateDetectedColor();
                    linefollowing(55);
                    updateDetectedColor();

                    if (detectedColor == 'R') { // 3rd position REDnm
                      delayWithoutBlocking(50);
                      stopMotors();
                      delayWithoutBlocking(500);
                      Backward_To_Line();
                      delayWithoutBlocking(500);
                      myServo.write(0);
                      delayWithoutBlocking(500);
                      myServoGripper.write(90);
                      delayWithoutBlocking(500);
                      myServo.write(120);
                      delayWithoutBlocking(500);
                      myServoGripper.write(10);
                      delayWithoutBlocking(500);
                      On_To_Line2();
                      delayWithoutBlocking(500);
                      Turning_180Left();
                      delayWithoutBlocking(500);
                      Turning_Right();
                      delayWithoutBlocking(500);
                      Turning_Left();
                      delayWithoutBlocking(500);
                      return;
                    }
                  } while (1);
                }
              } while (1);
            }
          } while (1);

          AllDone = 1;
        }



        else if (detectedColor == 'G' || detectedColor == 'B') { // pick - second red neweinm
          delayWithoutBlocking(50);
          stopMotors();
          delayWithoutBlocking(500);
          On_To_Line();
          delayWithoutBlocking(500);
          Turning_180Left();
          delayWithoutBlocking(500);
          Turning_Left2();
          delayWithoutBlocking(500);

          do {
            updateDetectedColor();
            linefollowing(55);
            updateDetectedColor();

            if (detectedColor == 'R') { // 3rd position ekaa RED nm
              delayWithoutBlocking(50);
              stopMotors();
              delayWithoutBlocking(500);
              Backward_To_Line();
              delayWithoutBlocking(500);
              myServoGripper.write(90);
              delayWithoutBlocking(500);
              myServo.write(0);
              delayWithoutBlocking(500);
              myServoGripper.write(10);
              delayWithoutBlocking(500);
              myServo.write(120);
              delayWithoutBlocking(500);
              On_To_Line2();
              delayWithoutBlocking(500);
              Turning_180Left();
              delayWithoutBlocking(500);
              Turning_Right();
              delayWithoutBlocking(500);
              Turning_Left();
             // red_detected = 0;
              // delayWithoutBlocking(500);
              // Turning_Right();
              delayWithoutBlocking(500);
              Pass_Black_Box();
              delayWithoutBlocking(500);
              Turning_Left3();
              delayWithoutBlocking(500);
              Turning_Right();
              delayWithoutBlocking(300);
              Turning_Left3();
              delayWithoutBlocking(500);
              Turning_Right();
              return;

              do {
                updateDetectedColor();
                linefollowing(55);
                updateDetectedColor();

                if (detectedColor == 'R') { // place - 1st position RED nm
                  delayWithoutBlocking(50);
                  stopMotors();
                  delayWithoutBlocking(500);
                  Backward_To_Line();
                  delayWithoutBlocking(500);
                  myServo.write(0);
                  delayWithoutBlocking(500);
                  myServoGripper.write(90);
                  delayWithoutBlocking(500);
                  myServo.write(120);
                  delayWithoutBlocking(500);
                  myServoGripper.write(10);
                  delayWithoutBlocking(500);
                  On_To_Line2();
                  delayWithoutBlocking(500);
                  Turning_180Left();
                  delayWithoutBlocking(500);
                  Turning_Left();
                  delayWithoutBlocking(500);
                  Turning_Right();
                  return;
                }
                else if (detectedColor == 'G' || detectedColor == 'B') { // 1st red neweinm
                  delayWithoutBlocking(50);
                  stopMotors();
                  delayWithoutBlocking(500);
                  On_To_Line();
                  delayWithoutBlocking(500);
                  Turning_180Left();
                  delayWithoutBlocking(500);
                  Turning_Left2();
                  delayWithoutBlocking(500);

                  do {
                    updateDetectedColor();
                    linefollowing(55);
                    updateDetectedColor();

                    if (detectedColor == 'R') { // 2nd position ek REDnm
                      delayWithoutBlocking(50);
                      stopMotors();
                      delayWithoutBlocking(500);
                      Backward_To_Line();
                      myServo.write(0);
                      delayWithoutBlocking(500);
                      myServoGripper.write(90);
                      delayWithoutBlocking(500);
                      myServo.write(120);
                      delayWithoutBlocking(500);
                      myServoGripper.write(10);
                      delayWithoutBlocking(500);
                      On_To_Line2();
                      delayWithoutBlocking(500);
                      Turning_180Left();
                      delayWithoutBlocking(500);
                      Stop_at_Black_Line();
                      return;
                    }
                    else if (detectedColor == 'G' || detectedColor == 'B') { // 2nd position ek RED neweinm
                      delayWithoutBlocking(50);
                      stopMotors();
                      delayWithoutBlocking(500);
                      On_To_Line();
                      delayWithoutBlocking(500);
                      Turning_180Left();
                      delayWithoutBlocking(500);
                      Turning_Left2();
                      delayWithoutBlocking(500);

                      do {
                        updateDetectedColor();
                        linefollowing(55);
                        updateDetectedColor();

                        if (detectedColor == 'R') { // 3rd eka RED nm
                          delayWithoutBlocking(50);
                          stopMotors();
                          delayWithoutBlocking(500);
                          Backward_To_Line();
                          delayWithoutBlocking(500);
                          myServo.write(0);
                          delayWithoutBlocking(500);
                          myServoGripper.write(90);
                          delayWithoutBlocking(500);
                          myServo.write(120);
                          delayWithoutBlocking(500);
                          myServoGripper.write(10);
                          delayWithoutBlocking(500);
                          On_To_Line2();
                          delayWithoutBlocking(500);
                          Turning_180Left();
                          delayWithoutBlocking(500);
                          Turning_Right();
                          delayWithoutBlocking(500);
                          Turning_Left();
                          delayWithoutBlocking(500);
                          return;
                        }
                      } while (1);
                    }
                  } while (1);
                }
              } while (1);
            }
          } while (1);

        }
      } while (1);

      AllDone = 1;
      
    }
    
    // if (AllDone ==1) {

    //   break;
    // }
  } while (true);

  //break ;
}





void Colour_Detected_Green () {

  int red_detected = 0 ;

  do {
    linefollowing(55);
    updateDetectedColor();

    if (detectedColor == 'G') {  // pick - 1st position GREENnm
      red_detected = 1;

      delayWithoutBlocking(50);
      stopMotors();
      delayWithoutBlocking(500);
      Backward_To_Line();
      delayWithoutBlocking(500);
      myServoGripper.write(90);
      delayWithoutBlocking(500);
      myServo.write(0);
      delayWithoutBlocking(500);
      myServoGripper.write(10);
      delayWithoutBlocking(500);
      myServo.write(120);
      delayWithoutBlocking(500);
      On_To_Line2();
      delayWithoutBlocking(500);
      Turning_180Left();
      delayWithoutBlocking(500);
      Turning_Left();
      delayWithoutBlocking(500);
      Turning_Right();
      delayWithoutBlocking(500);
      Pass_Black_Box();
      delayWithoutBlocking(500);
      Turning_Left3();
      delayWithoutBlocking(500);
      Turning_Right();
      delayWithoutBlocking(300);
      Turning_Left3();
      delayWithoutBlocking(500);
      Turning_Right();

      do {
        updateDetectedColor();
        linefollowing(55);
        updateDetectedColor();

        if (detectedColor == 'G') { // place - 1st position GREEnm
          delayWithoutBlocking(50);
          stopMotors();
          delayWithoutBlocking(500);
          Backward_To_Line();
          delayWithoutBlocking(500);
          myServo.write(0);
          delayWithoutBlocking(500);
          myServoGripper.write(90);
          delayWithoutBlocking(500);
          myServo.write(120);
          delayWithoutBlocking(500);
          myServoGripper.write(10);
          delayWithoutBlocking(500);
          On_To_Line2();
          delayWithoutBlocking(500);
          Turning_180Left();
          delayWithoutBlocking(500);
          Turning_Left();
          delayWithoutBlocking(500);
          Turning_Right();
          return;
        }
        else if (detectedColor == 'R' || detectedColor == 'B') { // 1st GREE neweinm
          delayWithoutBlocking(50);
          stopMotors();
          delayWithoutBlocking(500);
          On_To_Line();
          delayWithoutBlocking(500);
          Turning_180Left();
          delayWithoutBlocking(500);
          Turning_Left2();
          delayWithoutBlocking(500);

          do {
            updateDetectedColor();
            linefollowing(55);
            updateDetectedColor();

            if (detectedColor == 'G') { // 2nd GREENnm
              delayWithoutBlocking(50);
              stopMotors();
              delayWithoutBlocking(500);
              Backward_To_Line();
              delayWithoutBlocking(500);
              myServo.write(0);
              delayWithoutBlocking(500);
              myServoGripper.write(90);
              delayWithoutBlocking(500);
              myServo.write(120);
              delayWithoutBlocking(500);
              myServoGripper.write(10);
              delayWithoutBlocking(500);
              On_To_Line2();
              delayWithoutBlocking(500);
              Turning_180Left();
              delayWithoutBlocking(500);
              Stop_at_Black_Line();
              return;
            }
            else if (detectedColor == 'R' || detectedColor == 'B') { // 2nd GREEN neweinm
              delayWithoutBlocking(50);
              stopMotors();
              delayWithoutBlocking(500);
              // Backward_To_Line();
              // delayWithoutBlocking(500);
              On_To_Line();
              delayWithoutBlocking(500);
              Turning_180Left();
              delayWithoutBlocking(500);
              Turning_Left2();
              delayWithoutBlocking(500);

              do {
                updateDetectedColor();
                linefollowing(55);
                updateDetectedColor();

                if (detectedColor == 'G') { // 3rd GREENnm
                  delayWithoutBlocking(50);
                  stopMotors();
                  delayWithoutBlocking(500);
                  Backward_To_Line();
                  delayWithoutBlocking(500);
                  myServo.write(0);
                  delayWithoutBlocking(500);
                  myServoGripper.write(90);
                  delayWithoutBlocking(500);
                  myServo.write(120);
                  delayWithoutBlocking(500);
                  myServoGripper.write(10);
                  delayWithoutBlocking(500);
                  On_To_Line2();
                  delayWithoutBlocking(500);
                  Turning_180Left();
                  delayWithoutBlocking(500);
                  Turning_Right();
                  delayWithoutBlocking(500);
                  Turning_Left();
                  delayWithoutBlocking(500);
                  return;
                }
              } while (1);
            }
          } while (1);
        }
      } while (1);
    }



    else if (detectedColor == 'R' || detectedColor == 'B') { // pick - 1st position GREEN neweinm
      delayWithoutBlocking(50);
      stopMotors();
      delayWithoutBlocking(500);
      On_To_Line();
      delayWithoutBlocking(500);
      Turning_180Left();
      delayWithoutBlocking(500);
      Turning_Left2();
      delayWithoutBlocking(500);

      do {
        updateDetectedColor();
        linefollowing(55);
        updateDetectedColor();

        if (detectedColor == 'G') { // 2nd position GREENnm
          updateDetectedColor();
          //Serial.println("Detected color from camera: R");

          //red_detected = 1;
          delayWithoutBlocking(50);
          stopMotors();
          delayWithoutBlocking(500);
          Backward_To_Line();
          delayWithoutBlocking(500);
          myServoGripper.write(90);
          delayWithoutBlocking(500);
          myServo.write(0);
          delayWithoutBlocking(500);
          myServoGripper.write(10);
          delayWithoutBlocking(500);
          myServo.write(120);
          delayWithoutBlocking(500);
          On_To_Line2();
          delayWithoutBlocking(500);
          Turning_180Left();
          delayWithoutBlocking(500);
          Stop_at_Black_Line();
          delayWithoutBlocking(500);
          Pass_Black_Box();
          delayWithoutBlocking(500);
          Turning_Left3();
          delayWithoutBlocking(500);
          Turning_Right();
          delayWithoutBlocking(300);
          Turning_Left3();
          delayWithoutBlocking(500);
          Turning_Right();

          do {
            updateDetectedColor();
            linefollowing(55);
            updateDetectedColor();

            if (detectedColor == 'G') { // place - 1st GREENnm
              delayWithoutBlocking(50);
              stopMotors();
              delayWithoutBlocking(500);
              Backward_To_Line();
              delayWithoutBlocking(500);
              myServo.write(0);
              delayWithoutBlocking(500);
              myServoGripper.write(90);
              delayWithoutBlocking(500);
              myServo.write(120);
              delayWithoutBlocking(500);
              myServoGripper.write(10);
              delayWithoutBlocking(500);
              On_To_Line2(); 
              delayWithoutBlocking(500);
              Turning_180Left();
              delayWithoutBlocking(500);
              Turning_Left();
              delayWithoutBlocking(500);
              Turning_Right();
              return;
            }
            else if (detectedColor == 'R' || detectedColor == 'B') { // 1st GREEN neweinm
              delayWithoutBlocking(50);
              stopMotors();
              delayWithoutBlocking(500);
              On_To_Line();
              delayWithoutBlocking(500);
              Turning_180Left();
              delayWithoutBlocking(500);
              Turning_Left2();
              delayWithoutBlocking(500);

              do {
                updateDetectedColor();
                linefollowing(55);
                updateDetectedColor();

                if (detectedColor == 'G') { // 2nd place GREENnm
                  delayWithoutBlocking(50);
                  stopMotors();
                  delayWithoutBlocking(500);
                  Backward_To_Line();
                  delayWithoutBlocking(500);
                  myServo.write(0);
                  delayWithoutBlocking(500);
                  myServoGripper.write(90);
                  delayWithoutBlocking(500);
                  myServo.write(120);
                  delayWithoutBlocking(500);
                  myServoGripper.write(10);
                  delayWithoutBlocking(500);
                  On_To_Line2();
                  delayWithoutBlocking(500);
                  Turning_180Left();
                  delayWithoutBlocking(500);
                  Stop_at_Black_Line();
                  return;
                }
                else if (detectedColor == 'R' || detectedColor == 'B') { // 2nd place GREEN neweinm
                  delayWithoutBlocking(50);
                  stopMotors();
                  delayWithoutBlocking(500);
                  On_To_Line();
                  delayWithoutBlocking(500);
                  Turning_180Left();
                  delayWithoutBlocking(500);
                  Turning_Left2();
                  delayWithoutBlocking(500);

                  do {
                    updateDetectedColor();
                    linefollowing(55);
                    updateDetectedColor();

                    if (detectedColor == 'G') { // 3rd position GREENnm
                      delayWithoutBlocking(50);
                      stopMotors();
                      delayWithoutBlocking(500);
                      Backward_To_Line();
                      delayWithoutBlocking(500);
                      myServo.write(0);
                      delayWithoutBlocking(500);
                      myServoGripper.write(90);
                      delayWithoutBlocking(500);
                      myServo.write(120);
                      delayWithoutBlocking(500);
                      myServoGripper.write(10);
                      delayWithoutBlocking(500);
                      On_To_Line2();
                      delayWithoutBlocking(500);
                      Turning_180Left();
                      delayWithoutBlocking(500);
                      Turning_Right();
                      delayWithoutBlocking(500);
                      Turning_Left();
                      delayWithoutBlocking(500);
                      return;
                    }
                  } while (1);
                }
              } while (1);
            }
          } while (1);

          red_detected = 0;
        }



        else if (detectedColor == 'R' || detectedColor == 'B') { // pick - second GREEN neweinm
          delayWithoutBlocking(50);
          stopMotors();
          delayWithoutBlocking(500);
          On_To_Line();
          delayWithoutBlocking(500);
          Turning_180Left();
          delayWithoutBlocking(500);
          Turning_Left2();
          delayWithoutBlocking(500);

          do {
            updateDetectedColor();
            linefollowing(55);
            updateDetectedColor();

            if (detectedColor == 'G') { // 3rd position ekaa GREEN nm
              delayWithoutBlocking(50);
              stopMotors();
              delayWithoutBlocking(500);
              Backward_To_Line();
              delayWithoutBlocking(500);
              myServoGripper.write(90);
              delayWithoutBlocking(500);
              myServo.write(0);
              delayWithoutBlocking(500);
              myServoGripper.write(10);
              delayWithoutBlocking(500);
              myServo.write(120);
              delayWithoutBlocking(500);
              On_To_Line2();
              delayWithoutBlocking(500);
              Turning_180Left();
              delayWithoutBlocking(500);
              Turning_Right();
              delayWithoutBlocking(500);
              Turning_Left();
             // red_detected = 0;
              // delayWithoutBlocking(500);
              // Turning_Right();
              delayWithoutBlocking(500);
              Pass_Black_Box();
              delayWithoutBlocking(500);
              Turning_Left3();
              delayWithoutBlocking(500);
              Turning_Right();
              delayWithoutBlocking(300);
              Turning_Left3();
              delayWithoutBlocking(500);
              Turning_Right();

              do {
                updateDetectedColor();
                linefollowing(55);
                updateDetectedColor();

                if (detectedColor == 'G') { // place - 1st position GREEN nm
                  delayWithoutBlocking(50);
                  stopMotors();
                  delayWithoutBlocking(500);
                  Backward_To_Line();
                  delayWithoutBlocking(500);
                  myServo.write(0);
                  delayWithoutBlocking(500);
                  myServoGripper.write(90);
                  delayWithoutBlocking(500);
                  myServo.write(120);
                  delayWithoutBlocking(500);
                  myServoGripper.write(10);
                  delayWithoutBlocking(500);
                  On_To_Line2();
                  delayWithoutBlocking(500);
                  Turning_180Left();
                  delayWithoutBlocking(500);
                  Turning_Left();
                  delayWithoutBlocking(500);
                  Turning_Right();
                  return;
                }
                else if (detectedColor == 'R' || detectedColor == 'B') { // 1st GREEN neweinm
                  delayWithoutBlocking(50);
                  stopMotors();
                  delayWithoutBlocking(50);
                  On_To_Line();
                  delayWithoutBlocking(500);
                  Turning_180Left();
                  delayWithoutBlocking(500);
                  Turning_Left2();
                  delayWithoutBlocking(500);

                  do {
                    updateDetectedColor();
                    linefollowing(55);
                    updateDetectedColor();

                    if (detectedColor == 'G') { // 2nd position ek GREENnm
                      delayWithoutBlocking(50);
                      stopMotors();
                      delayWithoutBlocking(500);
                      Backward_To_Line();
                      delayWithoutBlocking(500);
                      myServo.write(0);
                      delayWithoutBlocking(500);
                      myServoGripper.write(90);
                      delayWithoutBlocking(500);
                      myServo.write(120);
                      delayWithoutBlocking(500);
                      myServoGripper.write(10);
                      delayWithoutBlocking(500);
                      On_To_Line2();
                      delayWithoutBlocking(500);
                      Turning_180Left();
                      delayWithoutBlocking(500);
                      Stop_at_Black_Line();
                      return;
                    }
                    else if (detectedColor == 'R' || detectedColor == 'B') { // 2nd position ek GREEN neweinm
                      delayWithoutBlocking(50);
                      stopMotors();
                      delayWithoutBlocking(500);
                      On_To_Line();
                      delayWithoutBlocking(500);
                      Turning_180Left();
                      delayWithoutBlocking(500);
                      Turning_Left2();
                      delayWithoutBlocking(500);

                      do {
                        updateDetectedColor();
                        linefollowing(55);
                        updateDetectedColor();

                        if (detectedColor == 'G') { // 3rd eka GREEN nm
                          delayWithoutBlocking(50);
                          stopMotors();
                          delayWithoutBlocking(500);
                          Backward_To_Line();
                          delayWithoutBlocking(500);
                          myServo.write(0);
                          delayWithoutBlocking(500);
                          myServoGripper.write(90);
                          delayWithoutBlocking(500);
                          myServo.write(120);
                          delayWithoutBlocking(500);
                          myServoGripper.write(10);
                          delayWithoutBlocking(500);
                          On_To_Line2();
                          delayWithoutBlocking(500);
                          Turning_180Left();
                          delayWithoutBlocking(500);
                          Turning_Right();
                          delayWithoutBlocking(500);
                          Turning_Left();
                          delayWithoutBlocking(500);
                          return;
                        }
                      } while (1);
                    }
                  } while (1);
                }
              } while (1);
            }
          } while (1);
        }
      } while (1);
    }
  } while (true);
}





// void Colour_Detected_Blue () {

//   int red_detected = 0 ;

//   do {
//     linefollowing(55);
//     updateDetectedColor();

//     if (detectedColor == 'B') {  // pick - 1st position BLUEnm
//       red_detected = 1;

//       delayWithoutBlocking(50);
//       stopMotors();
//       delayWithoutBlocking(500);
//       Backward_To_Line();
//       delayWithoutBlocking(500);
//       myServoGripper.write(90);
//       delayWithoutBlocking(500);
//       myServo.write(0);
//       delayWithoutBlocking(500);
//       myServoGripper.write(10);
//       delayWithoutBlocking(500);
//       myServo.write(120);
//       delayWithoutBlocking(500);
//       On_To_Line2();
//       delayWithoutBlocking(500);
//       Turning_180Left();
//       delayWithoutBlocking(500);
//       Turning_Left();
//       delayWithoutBlocking(500);
//       Turning_Right();
//       delayWithoutBlocking(500);
//       Pass_Black_Box();
//       delayWithoutBlocking(500);
//       Turning_Left3();
//       delayWithoutBlocking(500);
//       Turning_Right();
//       delayWithoutBlocking(300);
//       Turning_Left3();
//       delayWithoutBlocking(500);
//       Turning_Right();

//       do {
//         updateDetectedColor();
//         linefollowing(55);
//         updateDetectedColor();

//         if (detectedColor == 'B') { // place - 1st position BLUEnm
//           delayWithoutBlocking(50);
//           stopMotors();
//           delayWithoutBlocking(500);
//           Backward_To_Line();
//           delayWithoutBlocking(500);
//           myServo.write(0);
//           delayWithoutBlocking(500);
//           myServoGripper.write(90);
//           delayWithoutBlocking(500);
//           myServo.write(120);
//           delayWithoutBlocking(500);
//           myServoGripper.write(10);
//           delayWithoutBlocking(500);
//           On_To_Line2();
//           delayWithoutBlocking(500);
//           Turning_180Left();
//           delayWithoutBlocking(500);
//           Turning_Left();
//           delayWithoutBlocking(500);
//           Turning_Right();
//           return;
//         }
//         else if (detectedColor == 'R' || detectedColor == 'G') { // 1st BLUE neweinm
//           delayWithoutBlocking(50);
//           stopMotors();
//           delayWithoutBlocking(500);
//           On_To_Line2();
//           Turning_180Left();
//           delayWithoutBlocking(500);
//           Turning_Left2();
//           delayWithoutBlocking(500);

//           do {
//             updateDetectedColor();
//             linefollowing(55);
//             updateDetectedColor();

//             if (detectedColor == 'B') { // 2nd BLUEnm
//               delayWithoutBlocking(50);
//               stopMotors();
//               delayWithoutBlocking(500);
//               Backward_To_Line();
//               delayWithoutBlocking(500);
//               myServo.write(0);
//               delayWithoutBlocking(500);
//               myServoGripper.write(90);
//               delayWithoutBlocking(500);
//               myServo.write(120);
//               delayWithoutBlocking(500);
//               myServoGripper.write(10);
//               delayWithoutBlocking(500);
//               On_To_Line2();
//               delayWithoutBlocking(500);
//               Turning_180Left();
//               delayWithoutBlocking(500);
//               Stop_at_Black_Line();
//               return;
//             }
//             else if (detectedColor == 'R' || detectedColor == 'G') { // 2nd BLUE neweinm
//               delayWithoutBlocking(50);
//               stopMotors();
//               delayWithoutBlocking(500);
//               On_To_Line2();
//               delayWithoutBlocking(500);
//               Turning_180Left();
//               delayWithoutBlocking(500);
//               Turning_Left2();
//               delayWithoutBlocking(500);

//               do {
//                 updateDetectedColor();
//                 linefollowing(55);
//                 updateDetectedColor();

//                 if (detectedColor == 'B') { // 3rd REDnm
//                   delayWithoutBlocking(50);
//                   stopMotors();
//                   delayWithoutBlocking(500);
//                   Backward_To_Line();
//                   delayWithoutBlocking(500);
//                   myServo.write(0);
//                   delayWithoutBlocking(500);
//                   myServoGripper.write(90);
//                   delayWithoutBlocking(500);
//                   myServo.write(120);
//                   delayWithoutBlocking(500);
//                   myServoGripper.write(10);
//                   delayWithoutBlocking(500);
//                   Turning_180Left();
//                   delayWithoutBlocking(500);
//                   Turning_Right();
//                   delayWithoutBlocking(500);
//                   Turning_Left();
//                   delayWithoutBlocking(500);
//                   return;
//                 }
//               } while (1);
//             }
//           } while (1);
//         }
//       } while (1);
//     }



//     else if (detectedColor == 'R' || detectedColor == 'G') { // pick - 1st position BLUE neweinm
//       delayWithoutBlocking(50);
//       stopMotors();
//       delayWithoutBlocking(500);
//       On_To_Line2();
//       delayWithoutBlocking(500);
//       Turning_180Left();
//       delayWithoutBlocking(500);
//       Turning_Left2();
//       delayWithoutBlocking(500);

//       do {
//         updateDetectedColor();
//         linefollowing(55);
//         updateDetectedColor();

//         if (detectedColor == 'B') { // 2nd position BLUEnm
//           updateDetectedColor();
//           //Serial.println("Detected color from camera: R");

//           //red_detected = 1;
//           delayWithoutBlocking(50);
//           stopMotors();
//           delayWithoutBlocking(500);
//           Backward_To_Line();
//           delayWithoutBlocking(500);
//           myServoGripper.write(90);
//           delayWithoutBlocking(500);
//           myServo.write(0);
//           delayWithoutBlocking(500);
//           myServoGripper.write(10);
//           delayWithoutBlocking(500);
//           myServo.write(120);
//           delayWithoutBlocking(500);
//           On_To_Line2();
//           delayWithoutBlocking(500);
//           Turning_180Left();
//           delayWithoutBlocking(500);
//           Stop_at_Black_Line();
//           delayWithoutBlocking(500);
//           Pass_Black_Box();
//           delayWithoutBlocking(500);
//           Turning_Left3();
//           delayWithoutBlocking(500);
//           Turning_Right();
//           delayWithoutBlocking(300);
//           Turning_Left3();
//           delayWithoutBlocking(500);
//           Turning_Right();

//           do {
//             updateDetectedColor();
//             linefollowing(55);
//             updateDetectedColor();

//             if (detectedColor == 'B') { // place - 1st BLUEnm
//               delayWithoutBlocking(50);
//               stopMotors();
//               delayWithoutBlocking(500);
//               Backward_To_Line();
//               delayWithoutBlocking(500);
//               Backward_To_Line();
//               delayWithoutBlocking(500);
//               myServo.write(0);
//               delayWithoutBlocking(500);
//               myServoGripper.write(90);
//               delayWithoutBlocking(500);
//               myServo.write(120);
//               delayWithoutBlocking(500);
//               myServoGripper.write(10);
//               delayWithoutBlocking(500);
//               Turning_180Left();
//               delayWithoutBlocking(500);
//               Turning_Left();
//               delayWithoutBlocking(500);
//               Turning_Right();
//               return;
//             }
//             else if (detectedColor == 'R' || detectedColor == 'G') { // 1st RED neweinm
//               delayWithoutBlocking(50);
//               stopMotors();
//               delayWithoutBlocking(500);
//               Turning_180Left();
//               delayWithoutBlocking(500);
//               Turning_Left2();
//               delayWithoutBlocking(500);

//               do {
//                 updateDetectedColor();
//                 linefollowing(55);
//                 updateDetectedColor();

//                 if (detectedColor == 'B') { // 2nd place BLUEnm
//                   delayWithoutBlocking(50);
//                   stopMotors();
//                   delayWithoutBlocking(500);
//                   Backward_To_Line();
//                   delayWithoutBlocking(500);
//                   myServo.write(0);
//                   delayWithoutBlocking(500);
//                   myServoGripper.write(90);
//                   delayWithoutBlocking(500);
//                   myServo.write(120);
//                   delayWithoutBlocking(500);
//                   myServoGripper.write(10);
//                   delayWithoutBlocking(500);
//                   Turning_180Left();
//                   delayWithoutBlocking(500);
//                   Stop_at_Black_Line();
//                   return;
//                 }
//                 else if (detectedColor == 'R' || detectedColor == 'G') { // 2nd place BLUE neweinm
//                   delayWithoutBlocking(50);
//                   stopMotors();
//                   Turning_180Left();
//                   delayWithoutBlocking(500);
//                   Turning_Left2();
//                   delayWithoutBlocking(500);

//                   do {
//                     updateDetectedColor();
//                     linefollowing(55);
//                     updateDetectedColor();

//                     if (detectedColor == 'B') { // 3rd position BLUEnm
//                       delayWithoutBlocking(50);
//                       stopMotors();
//                       delayWithoutBlocking(500);
//                       Backward_To_Line();
//                       delayWithoutBlocking(500);
//                       myServo.write(0);
//                       delayWithoutBlocking(500);
//                       myServoGripper.write(90);
//                       delayWithoutBlocking(500);
//                       myServo.write(120);
//                       delayWithoutBlocking(500);
//                       myServoGripper.write(10);
//                       delayWithoutBlocking(500);
//                       Turning_180Left();
//                       delayWithoutBlocking(500);
//                       Turning_Right();
//                       delayWithoutBlocking(500);
//                       Turning_Left();
//                       delayWithoutBlocking(500);
//                       return;
//                     }
//                   } while (1);
//                 }
//               } while (1);
//             }
//           } while (1);

//           red_detected = 0;
//         }



//         else if (detectedColor == 'R' || detectedColor == 'G') { // pick - second BLUE neweinm
//           delayWithoutBlocking(50);
//           stopMotors();
//           delayWithoutBlocking(500);
//           On_To_Line2();
//           delayWithoutBlocking(500);
//           Turning_180Left();
//           delayWithoutBlocking(500);
//           Turning_Left2();
//           delayWithoutBlocking(500);

//           do {
//             updateDetectedColor();
//             linefollowing(55);
//             updateDetectedColor();

//             if (detectedColor == 'B') { // 3rd position ekaa BLUE nm
//               delayWithoutBlocking(50);
//               stopMotors();
//               delayWithoutBlocking(500);
//               Backward_To_Line();
//               delayWithoutBlocking(500);
//               myServoGripper.write(90);
//               delayWithoutBlocking(500);
//               myServo.write(0);
//               delayWithoutBlocking(500);
//               myServoGripper.write(10);
//               delayWithoutBlocking(500);
//               myServo.write(120);
//               delayWithoutBlocking(500);
//               On_To_Line2();
//               delayWithoutBlocking(500);
//               Turning_180Left();
//               delayWithoutBlocking(500);
//               Turning_Right();
//               delayWithoutBlocking(500);
//               Turning_Left();
//               delayWithoutBlocking(500);
//               Pass_Black_Box();
//               delayWithoutBlocking(500);
//               Turning_Left3();
//               delayWithoutBlocking(500);
//               Turning_Right();
//               delayWithoutBlocking(300);
//               Turning_Left3();
//               delayWithoutBlocking(500);
//               Turning_Right();

//               do {
//                 updateDetectedColor();
//                 linefollowing(55);
//                 updateDetectedColor();

//                 if (detectedColor == 'B') { // place - 1st position BLUE nm
//                   delayWithoutBlocking(50);
//                   stopMotors();
//                   delayWithoutBlocking(500);
//                   Backward_To_Line();
//                   delayWithoutBlocking(500);
//                   myServo.write(0);
//                   delayWithoutBlocking(500);
//                   myServoGripper.write(90);
//                   delayWithoutBlocking(500);
//                   myServo.write(120);
//                   delayWithoutBlocking(500);
//                   myServoGripper.write(10);
//                   delayWithoutBlocking(500);
//                   Turning_180Left();
//                   delayWithoutBlocking(500);
//                   Turning_Left();
//                   delayWithoutBlocking(500);
//                   Turning_Right();
//                   return;
//                 }
//                 else if (detectedColor == 'R' || detectedColor == 'G') { // 1st BLUE neweinm
//                   delayWithoutBlocking(50);
//                   stopMotors();
//                   delayWithoutBlocking(50);
//                   Turning_180Left();
//                   delayWithoutBlocking(500);
//                   Turning_Left2();
//                   delayWithoutBlocking(500);

//                   do {
//                     updateDetectedColor();
//                     linefollowing(55);
//                     updateDetectedColor();

//                     if (detectedColor == 'B') { // 2nd position ek BLUEnm
//                       delayWithoutBlocking(50);
//                       stopMotors();
//                       delayWithoutBlocking(500);
//                       Backward_To_Line();
//                       myServo.write(0);
//                       delayWithoutBlocking(500);
//                       myServoGripper.write(90);
//                       delayWithoutBlocking(500);
//                       myServo.write(120);
//                       delayWithoutBlocking(500);
//                       myServoGripper.write(10);
//                       delayWithoutBlocking(500);
//                       Turning_180Left();
//                       delayWithoutBlocking(500);
//                       Stop_at_Black_Line();
//                       return;
//                     }
//                     else if (detectedColor == 'R' || detectedColor == 'G') { // 2nd position ek BLUE neweinm
//                       delayWithoutBlocking(50);
//                       stopMotors();
//                       delayWithoutBlocking(500);
//                       Turning_180Left();
//                       delayWithoutBlocking(500);
//                       Turning_Left2();
//                       delayWithoutBlocking(500);

//                       do {
//                         updateDetectedColor();
//                         linefollowing(55);
//                         updateDetectedColor();

//                         if (detectedColor == 'B') { // 3rd eka BLUE nm
//                           delayWithoutBlocking(50);
//                           stopMotors();
//                           delayWithoutBlocking(500);
//                           Backward_To_Line();
//                           delayWithoutBlocking(500);
//                           myServo.write(0);
//                           delayWithoutBlocking(500);
//                           myServoGripper.write(90);
//                           delayWithoutBlocking(500);
//                           myServo.write(120);
//                           delayWithoutBlocking(500);
//                           myServoGripper.write(10);
//                           delayWithoutBlocking(500);
//                           Turning_180Left();
//                           delayWithoutBlocking(500);
//                           Turning_Right();
//                           delayWithoutBlocking(500);
//                           Turning_Left();
//                           delayWithoutBlocking(500);
//                           return;
//                         }
//                       } while (1);
//                     }
//                   } while (1);
//                 }
//               } while (1);
//             }
//           } while (1);
//         }
//       } while (1);
//     }
//   } while (true);
// }






void Colour_Detected_Blue () {

  int red_detected = 0 ;

  do {
    linefollowing(55);
    updateDetectedColor();

    if (detectedColor == 'B') {  // pick - 1st position BLUEnm
      red_detected = 1;

      delayWithoutBlocking(50);
      stopMotors();
      delayWithoutBlocking(500);
      Backward_To_Line();
      delayWithoutBlocking(500);
      myServoGripper.write(90);
      delayWithoutBlocking(500);
      myServo.write(0);
      delayWithoutBlocking(500);
      myServoGripper.write(10);
      delayWithoutBlocking(500);
      myServo.write(120);
      delayWithoutBlocking(500);
      On_To_Line2();
      delayWithoutBlocking(500);
      Turning_180Left();
      delayWithoutBlocking(500);
      Turning_Left();
      delayWithoutBlocking(500);
      Turning_Right();
      delayWithoutBlocking(500);
      Pass_Black_Box();
      delayWithoutBlocking(500);
      Turning_Left3();
      delayWithoutBlocking(500);
      Turning_Right();
      delayWithoutBlocking(300);
      Turning_Left3();
      delayWithoutBlocking(500);
      Turning_Right();

      do {
        updateDetectedColor();
        linefollowing(55);
        updateDetectedColor();

        if (detectedColor == 'B') { // place - 1st position BLUEnm
          delayWithoutBlocking(50);
          stopMotors();
          delayWithoutBlocking(500);
          Backward_To_Line();
          delayWithoutBlocking(500);
          myServo.write(0);
          delayWithoutBlocking(500);
          myServoGripper.write(90);
          delayWithoutBlocking(500);
          myServo.write(120);
          delayWithoutBlocking(500);
          myServoGripper.write(10);
          delayWithoutBlocking(500);
          On_To_Line2();
          delayWithoutBlocking(500);
          Turning_180Left();
          delayWithoutBlocking(500);
          Turning_Left();
          delayWithoutBlocking(500);
          Turning_Right();
          return;
        }
        else if (detectedColor == 'R' || detectedColor == 'G') { // 1st BLUE neweinm
          delayWithoutBlocking(50);
          stopMotors();
          delayWithoutBlocking(500);
          On_To_Line();
          delayWithoutBlocking(500);
          Turning_180Left();
          delayWithoutBlocking(500);
          Turning_Left2();
          delayWithoutBlocking(500);

          do {
            updateDetectedColor();
            linefollowing(55);
            updateDetectedColor();

            if (detectedColor == 'B') { // 2nd BLUEnm
              delayWithoutBlocking(50);
              stopMotors();
              delayWithoutBlocking(500);
              Backward_To_Line();
              delayWithoutBlocking(500);
              myServo.write(0);
              delayWithoutBlocking(500);
              myServoGripper.write(90);
              delayWithoutBlocking(500);
              myServo.write(120);
              delayWithoutBlocking(500);
              myServoGripper.write(10);
              delayWithoutBlocking(500);
              On_To_Line2();
              delayWithoutBlocking(500);
              Turning_180Left();
              delayWithoutBlocking(500);
              Stop_at_Black_Line();
              return;
            }
            else if (detectedColor == 'R' || detectedColor == 'G') { // 2nd BLUE neweinm
              delayWithoutBlocking(50);
              stopMotors();
              delayWithoutBlocking(500);
              // Backward_To_Line();
              // delayWithoutBlocking(500);
              On_To_Line();
              delayWithoutBlocking(500);
              Turning_180Left();
              delayWithoutBlocking(500);
              Turning_Left2();
              delayWithoutBlocking(500);

              do {
                updateDetectedColor();
                linefollowing(55);
                updateDetectedColor();

                if (detectedColor == 'B') { // 3rd BLUEnm
                  delayWithoutBlocking(50);
                  stopMotors();
                  delayWithoutBlocking(500);
                  Backward_To_Line();
                  delayWithoutBlocking(500);
                  myServo.write(0);
                  delayWithoutBlocking(500);
                  myServoGripper.write(90);
                  delayWithoutBlocking(500);
                  myServo.write(120);
                  delayWithoutBlocking(500);
                  myServoGripper.write(10);
                  delayWithoutBlocking(500);
                  On_To_Line2();
                  delayWithoutBlocking(500);
                  Turning_180Left();
                  delayWithoutBlocking(500);
                  Turning_Right();
                  delayWithoutBlocking(500);
                  Turning_Left();
                  delayWithoutBlocking(500);
                  return;
                }
              } while (1);
            }
          } while (1);
        }
      } while (1);
    }



    else if (detectedColor == 'R' || detectedColor == 'G') { // pick - 1st position BLUE neweinm
      delayWithoutBlocking(50);
      stopMotors();
      delayWithoutBlocking(500);
      On_To_Line();
      delayWithoutBlocking(500);
      Turning_180Left();
      delayWithoutBlocking(500);
      Turning_Left2();
      delayWithoutBlocking(500);

      do {
        updateDetectedColor();
        linefollowing(55);
        updateDetectedColor();

        if (detectedColor == 'B') { // 2nd position BLUEnm
          updateDetectedColor();
          //Serial.println("Detected color from camera: R");

          //red_detected = 1;
          delayWithoutBlocking(50);
          stopMotors();
          delayWithoutBlocking(500);
          Backward_To_Line();
          delayWithoutBlocking(500);
          myServoGripper.write(90);
          delayWithoutBlocking(500);
          myServo.write(0);
          delayWithoutBlocking(500);
          myServoGripper.write(10);
          delayWithoutBlocking(500);
          myServo.write(120);
          delayWithoutBlocking(500);
          On_To_Line2();
          delayWithoutBlocking(500);
          Turning_180Left();
          delayWithoutBlocking(500);
          Stop_at_Black_Line();
          delayWithoutBlocking(500);
          Pass_Black_Box();
          delayWithoutBlocking(500);
          Turning_Left3();
          delayWithoutBlocking(500);
          Turning_Right();
          delayWithoutBlocking(300);
          Turning_Left3();
          delayWithoutBlocking(500);
          Turning_Right();

          do {
            updateDetectedColor();
            linefollowing(55);
            updateDetectedColor();

            if (detectedColor == 'B') { // place - 1st BLUEnm
              delayWithoutBlocking(50);
              stopMotors();
              delayWithoutBlocking(500);
              Backward_To_Line();
              delayWithoutBlocking(500);
              myServo.write(0);
              delayWithoutBlocking(500);
              myServoGripper.write(90);
              delayWithoutBlocking(500);
              myServo.write(120);
              delayWithoutBlocking(500);
              myServoGripper.write(10);
              delayWithoutBlocking(500);
              On_To_Line2(); 
              delayWithoutBlocking(500);
              Turning_180Left();
              delayWithoutBlocking(500);
              Turning_Left();
              delayWithoutBlocking(500);
              Turning_Right();
              return;
            }
            else if (detectedColor == 'R' || detectedColor == 'G') { // 1st BLUE neweinm
              delayWithoutBlocking(50);
              stopMotors();
              delayWithoutBlocking(500);
              On_To_Line();
              delayWithoutBlocking(500);
              Turning_180Left();
              delayWithoutBlocking(500);
              Turning_Left2();
              delayWithoutBlocking(500);

              do {
                updateDetectedColor();
                linefollowing(55);
                updateDetectedColor();

                if (detectedColor == 'B') { // 2nd place BLUEnm
                  delayWithoutBlocking(50);
                  stopMotors();
                  delayWithoutBlocking(500);
                  Backward_To_Line();
                  delayWithoutBlocking(500);
                  myServo.write(0);
                  delayWithoutBlocking(500);
                  myServoGripper.write(90);
                  delayWithoutBlocking(500);
                  myServo.write(120);
                  delayWithoutBlocking(500);
                  myServoGripper.write(10);
                  delayWithoutBlocking(500);
                  On_To_Line2();
                  delayWithoutBlocking(500);
                  Turning_180Left();
                  delayWithoutBlocking(500);
                  Stop_at_Black_Line();
                  return;
                }
                else if (detectedColor == 'R' || detectedColor == 'G') { // 2nd place BLUE neweinm
                  delayWithoutBlocking(50);
                  stopMotors();
                  delayWithoutBlocking(500);
                  On_To_Line();
                  delayWithoutBlocking(500);
                  Turning_180Left();
                  delayWithoutBlocking(500);
                  Turning_Left2();
                  delayWithoutBlocking(500);

                  do {
                    updateDetectedColor();
                    linefollowing(55);
                    updateDetectedColor();

                    if (detectedColor == 'B') { // 3rd position BLUEnm
                      delayWithoutBlocking(50);
                      stopMotors();
                      delayWithoutBlocking(500);
                      Backward_To_Line();
                      delayWithoutBlocking(500);
                      myServo.write(0);
                      delayWithoutBlocking(500);
                      myServoGripper.write(90);
                      delayWithoutBlocking(500);
                      myServo.write(120);
                      delayWithoutBlocking(500);
                      myServoGripper.write(10);
                      delayWithoutBlocking(500);
                      On_To_Line2();
                      delayWithoutBlocking(500);
                      Turning_180Left();
                      delayWithoutBlocking(500);
                      Turning_Right();
                      delayWithoutBlocking(500);
                      Turning_Left();
                      delayWithoutBlocking(500);
                      return;
                    }
                  } while (1);
                }
              } while (1);
            }
          } while (1);

          red_detected = 0;
        }



        else if (detectedColor == 'R' || detectedColor == 'G') { // pick - second BLUE neweinm
          delayWithoutBlocking(50);
          stopMotors();
          delayWithoutBlocking(500);
          On_To_Line();
          delayWithoutBlocking(500);
          Turning_180Left();
          delayWithoutBlocking(500);
          Turning_Left2();
          delayWithoutBlocking(500);

          do {
            updateDetectedColor();
            linefollowing(55);
            updateDetectedColor();

            if (detectedColor == 'B') { // 3rd position ekaa BLUE nm
              delayWithoutBlocking(50);
              stopMotors();
              delayWithoutBlocking(500);
              Backward_To_Line();
              delayWithoutBlocking(500);
              myServoGripper.write(90);
              delayWithoutBlocking(500);
              myServo.write(0);
              delayWithoutBlocking(500);
              myServoGripper.write(10);
              delayWithoutBlocking(500);
              myServo.write(120);
              delayWithoutBlocking(500);
              On_To_Line2();
              delayWithoutBlocking(500);
              Turning_180Left();
              delayWithoutBlocking(500);
              Turning_Right();
              delayWithoutBlocking(500);
              Turning_Left();
             // red_detected = 0;
              // delayWithoutBlocking(500);
              // Turning_Right();
              delayWithoutBlocking(500);
              Pass_Black_Box();
              delayWithoutBlocking(500);
              Turning_Left3();
              delayWithoutBlocking(500);
              Turning_Right();
              delayWithoutBlocking(300);
              Turning_Left3();
              delayWithoutBlocking(500);
              Turning_Right();

              do {
                updateDetectedColor();
                linefollowing(55);
                updateDetectedColor();

                if (detectedColor == 'B') { // place - 1st position BLUE nm
                  delayWithoutBlocking(50);
                  stopMotors();
                  delayWithoutBlocking(500);
                  Backward_To_Line();
                  delayWithoutBlocking(500);
                  myServo.write(0);
                  delayWithoutBlocking(500);
                  myServoGripper.write(90);
                  delayWithoutBlocking(500);
                  myServo.write(120);
                  delayWithoutBlocking(500);
                  myServoGripper.write(10);
                  delayWithoutBlocking(500);
                  On_To_Line2();
                  delayWithoutBlocking(500);
                  Turning_180Left();
                  delayWithoutBlocking(500);
                  Turning_Left();
                  delayWithoutBlocking(500);
                  Turning_Right();
                  return;
                }
                else if (detectedColor == 'R' || detectedColor == 'G') { // 1st BLUE neweinm
                  delayWithoutBlocking(50);
                  stopMotors();
                  delayWithoutBlocking(50);
                  On_To_Line();
                  delayWithoutBlocking(500);
                  Turning_180Left();
                  delayWithoutBlocking(500);
                  Turning_Left2();
                  delayWithoutBlocking(500);

                  do {
                    updateDetectedColor();
                    linefollowing(55);
                    updateDetectedColor();

                    if (detectedColor == 'B') { // 2nd position ek BLUEnm
                      delayWithoutBlocking(50);
                      stopMotors();
                      delayWithoutBlocking(500);
                      Backward_To_Line();
                      delayWithoutBlocking(500);
                      myServo.write(0);
                      delayWithoutBlocking(500);
                      myServoGripper.write(90);
                      delayWithoutBlocking(500);
                      myServo.write(120);
                      delayWithoutBlocking(500);
                      myServoGripper.write(10);
                      delayWithoutBlocking(500);
                      On_To_Line2();
                      delayWithoutBlocking(500);
                      Turning_180Left();
                      delayWithoutBlocking(500);
                      Stop_at_Black_Line();
                      return;
                    }
                    else if (detectedColor == 'R' || detectedColor == 'G') { // 2nd position ek BLUE neweinm
                      delayWithoutBlocking(50);
                      stopMotors();
                      delayWithoutBlocking(500);
                      On_To_Line();
                      delayWithoutBlocking(500);
                      Turning_180Left();
                      delayWithoutBlocking(500);
                      Turning_Left2();
                      delayWithoutBlocking(500);

                      do {
                        updateDetectedColor();
                        linefollowing(55);
                        updateDetectedColor();

                        if (detectedColor == 'B') { // 3rd eka BLUE nm
                          delayWithoutBlocking(50);
                          stopMotors();
                          delayWithoutBlocking(500);
                          Backward_To_Line();
                          delayWithoutBlocking(500);
                          myServo.write(0);
                          delayWithoutBlocking(500);
                          myServoGripper.write(90);
                          delayWithoutBlocking(500);
                          myServo.write(120);
                          delayWithoutBlocking(500);
                          myServoGripper.write(10);
                          delayWithoutBlocking(500);
                          On_To_Line2();
                          delayWithoutBlocking(500);
                          Turning_180Left();
                          delayWithoutBlocking(500);
                          Turning_Right();
                          delayWithoutBlocking(500);
                          Turning_Left();
                          delayWithoutBlocking(500);
                          return;
                        }
                      } while (1);
                    }
                  } while (1);
                }
              } while (1);
            }
          } while (1);
        }
      } while (1);
    }
  } while (true);
}





void On_To_Line() {
  set_forward();         // Set both motors to move forward
  analogWrite(ENA, 80);  // Left motor speed
  analogWrite(ENB, 58);  // Right motor speed
  delay(320);             // Move forward for 50ms
  stopMotors();          // Stop the robot
}





void On_To_Line2() {
  set_forward();         // Set both motors to move forward
  analogWrite(ENA, 80);  // Left motor speed
  analogWrite(ENB, 58);  // Right motor speed
  delay(550);             // Move forward for 50ms
  stopMotors();          // Stop the robot
}




void Backward_To_Line() {
  set_backward();         // Set both motors to move forward
  analogWrite(ENA, 80);  // Left motor speed
  analogWrite(ENB, 58);  // Right motor speed
  delay(270);             // Move forward for 50ms
  stopMotors();          // Stop the robot
}





void Pass_Black_Box (){

  do {

  linefollowing(50);

    // while (true) {

    read_IR();  // Update IR_VAL[6] with sensor values

    if (IR_VAL[0] == 1 && IR_VAL[1] == 1 && IR_VAL[2] == 1 && IR_VAL[3] == 1 && IR_VAL[4] == 1 && IR_VAL[5] == 1) {  // Sensor 3 or 4
        set_forward();         // Set both motors to move forward
        analogWrite(ENA, 70);  // Left motor speed
        analogWrite(ENB, 58);  // Right motor speed
        delay(1100);
        break;
    }
  // }


  } while (true);
}





void Pass_Black_Box_2 (){

  do {

  linefollowing(50);

    // while (true) {

    read_IR();  // Update IR_VAL[6] with sensor values

    if (IR_VAL[0] == 1 && IR_VAL[1] == 1 && IR_VAL[2] == 1 && IR_VAL[3] == 1 && IR_VAL[4] == 1 && IR_VAL[5] == 1) {  // Sensor 3 or 4
        set_forward();         // Set both motors to move forward
        analogWrite(ENA, 70);  // Left motor speed
        analogWrite(ENB, 58);  // Right motor speed
        delay(1600);
        break;
    }
  // }


  } while (true);
}




void END (){

  do {

  linefollowing(50);

    // while (true) {

    read_IR();  // Update IR_VAL[6] with sensor values

    if (IR_VAL[0] == 1 && IR_VAL[1] == 1 && IR_VAL[2] == 1 && IR_VAL[3] == 1 && IR_VAL[4] == 1 && IR_VAL[5] == 1) {  // Sensor 3 or 4
        set_forward();         // Set both motors to move forward
        analogWrite(ENA, 70);  // Left motor speed
        analogWrite(ENB, 58);  // Right motor speed
        delay(300);
        stopMotors();
        delay(30000);
        break;
    }
  // }


  } while (true);
}





void Stop_at_Black_Line (){

  do {

  linefollowing(65);

    // while (true) {

    read_IR();  // Update IR_VAL[6] with sensor values

    if (IR_VAL[0] == 1 && IR_VAL[1] == 1 && IR_VAL[2] == 1 && IR_VAL[3] == 1 && IR_VAL[4] == 1 && IR_VAL[5] == 1) {  // Sensor 3 or 4
        set_forward();         // Set both motors to move forward
        analogWrite(ENA, 70);  // Left motor speed
        analogWrite(ENB, 58);  // Right motor speed
        delay(40);
        stopMotors();
        break;
    }
  // }
  } while (true);
}





void Start() {


  // Stop right motor and spin left motor in reverse to make a sharp left turn 
  set_forward();         // Set both motors to move forward
  analogWrite(ENA, 65);  // Left motor speed
  analogWrite(ENB, 58);  // Right motor speed
  delay(1500);             // Move forward for 50ms
  stopMotors(); 
  return;
}





void Turning_Left() {

  int junction_count = 0;

 // firstTimeLineFollow = 1;
  do {

    linefollowing(65);
    
  leftTurnDetected = digitalRead(IR_LEFT_SENSOR_PIN);
    
    if (leftTurnDetected) {
     // firstTimeLineFollow = 1;
      junction_count++;
      //delay(300);
      stopMotors();
      delay(500);
      //Immidiate_Stop();
      On_To_Line();
      stopMotors();
      delay(500);
     // Immidiate_Stop();
    //sharpLeftTurn();
      Turning_90_Left();
      delay(500);
      
      if (junction_count >= 1) {
        //Immidiate_Stop();
        delay(50);
        break;
      }
      //break;
    }
  } while (true);
}





void Turning_Left2() {

  int junction_count = 0;

 // firstTimeLineFollow = 1;
  do {

    linefollowing(63);
    
  leftTurnDetected = digitalRead(IR_LEFT_SENSOR_PIN);
    
    if (leftTurnDetected) {
     // firstTimeLineFollow = 1;
      junction_count++;
      //delay(200);
      stopMotors();
      delay(500);
      //Immidiate_Stop();
      On_To_Line();
      stopMotors();
      delay(500);
     // Immidiate_Stop();
    //sharpLeftTurn();
      Turning_90_Left();
      delay(500);
      
      if (junction_count >= 2) {
        //Immidiate_Stop();
        delay(50);
        break;
      }
      //break;
    }
  } while (true);
}





void Turning_Left3() {

  int junction_count = 0;

 // firstTimeLineFollow = 1;
  do {

    linefollowing(65);
    
  leftTurnDetected = digitalRead(IR_LEFT_SENSOR_PIN);
    
    if (leftTurnDetected) {
     // firstTimeLineFollow = 1;
      junction_count++;
      //delay(200);
      stopMotors();
      delay(500);
      //Immidiate_Stop();
      On_To_Line();
      stopMotors();
      delay(500);
     // Immidiate_Stop();
    //sharpLeftTurn();
      Turning_90_Left();
      delay(500);
      
      if (junction_count >= 3) {
        //Immidiate_Stop();
        delay(50);
        break;
      }
      //break;
    }
  } while (true);
}





void Turning_Right() {

  int junction_count = 0;

 // firstTimeLineFollow = 1;
  do {

    linefollowing(65);
    
  rightTurnDetected = digitalRead(IR_RIGHT_SENSOR_PIN);
    
    if (rightTurnDetected) {
     // firstTimeLineFollow = 1;
      junction_count++;
      //delay(200);
      stopMotors();
      delay(500);
      //Immidiate_Stop();
      On_To_Line();
      stopMotors();
      delay(500);
     // Immidiate_Stop();
      Turning_90_Right();
      delay(500);
      
      if (junction_count >= 1) {
        //Immidiate_Stop();
        delay(50);
        break;
      }
      //break;
    }
  } while (true);
}




void Turning_Right2() {

  int junction_count = 0;

 // firstTimeLineFollow = 1;
  do {

    linefollowing(65);
    
  rightTurnDetected = digitalRead(IR_RIGHT_SENSOR_PIN);
    
    if (rightTurnDetected) {
     // firstTimeLineFollow = 1;
      junction_count++;
      //delay(200);
      stopMotors();
      delay(500);
      //Immidiate_Stop();
      On_To_Line();
      stopMotors();
      delay(500);
     // Immidiate_Stop();
      Turning_90_Right();
      delay(500);
      
      if (junction_count >= 2) {
        //Immidiate_Stop();
        delay(50);
        break;
      }
      //break;
    }
  } while (true);
}





void Turning_Right3() {

  int junction_count = 0;

 // firstTimeLineFollow = 1;
  do {

    linefollowing(65);
    
  rightTurnDetected = digitalRead(IR_RIGHT_SENSOR_PIN);
    
    if (rightTurnDetected) {
     // firstTimeLineFollow = 1;
      junction_count++;
      //delay(200);
      stopMotors();
      delay(500);
      //Immidiate_Stop();
      On_To_Line();
      stopMotors();
      delay(500);
     // Immidiate_Stop();
      Turning_90_Right();
      delay(500);
      
      if (junction_count >= 3) {
        //Immidiate_Stop();
        delay(50);
        break;
      }
      //break;
    }
  } while (true);
}





void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}





void Turning_90_Left() {

  // Begin sharp left turn
  digitalWrite(IN1, HIGH);  // Left motor forward
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);   // Right motor backward
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, MotorBasespeed);
  analogWrite(ENB, MotorBasespeed);

  delay(400);

  // Continue turning until middle sensors (3 or 4) detect line
  while (true) {
    read_IR();  // Update IR_VAL[6] with sensor values

    if (IR_VAL[3] == 1 && IR_VAL[4] == 1) {  // Sensor 3 or 4
      break;
    }
  }

  stopMotors();  // Stop motors after line detected
}





void Turning_180Left() {
  // Start turning: left motor backward, right motor forward
  digitalWrite(IN1, LOW);   // Left motor backward
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);  // Right motor forward
  digitalWrite(IN4, LOW);

  analogWrite(ENA, MotorBasespeed);
  analogWrite(ENB, MotorBasespeed);

  // Initial fixed 2-second rotation
  delay(400);

  // Continue rotating until middle sensors detect the line
  while (true) {
    read_IR();  // Update IR_VAL[6] with sensor readings

    // Check middle sensors: IR_VAL[2] = A3, IR_VAL[3] = A2
    if (IR_VAL[1] == 1 && IR_VAL[2] == 1) {
      break;
    }
  }

  stopMotors(); // Stop after detecting the line
}




void Turning_90_Right() {
  // Begin sharp right turn
  digitalWrite(IN1, LOW);   // Left motor backward
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);  // Right motor forward
  digitalWrite(IN4, LOW);

  analogWrite(ENA, MotorBasespeed);
  analogWrite(ENB, MotorBasespeed);

  delay(400);

  // Keep turning until center sensors detect the line
  while (true) {
    read_IR();  // Read sensor values

    if (IR_VAL[1] == 1 || IR_VAL[2] == 1) {  // Middle sensors A3 (index 2) or A2 (index 3)
      break;
    }
  }

  stopMotors();  // Stop once line is detected
}





//---------------- PID CONTROL ----------------
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