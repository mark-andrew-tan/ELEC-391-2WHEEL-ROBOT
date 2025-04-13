#include <PID_v1.h>
#include "Arduino_BMI270_BMM150.h"
#include <ArduinoBLE.h>
#include <Servo.h>
#include <Arduino.h>
#include <DFRobotDFPlayerMini.h>


#define BIN1 2  // Motor input 1
#define BIN2 3  // Motor input 1
#define AIN2 4  // Motor input 2 (left)
#define AIN1 5  // Motor input 2

#define BUTTON_SEND_VALUES 9  // Button to send the values over Bluetooth
#define BUFFER_SIZE 20

#define SERVO_PIN 10  
#define SERVO2_PIN 8

DFRobotDFPlayerMini myDFPlayer;

// // Define a custom BLE service and characteristic
BLEService customService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");
BLECharacteristic customCharacteristic("00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite | BLENotify, BUFFER_SIZE, false);

bool systemEnabled = false;

// IR SENSOR ------------------------------

const int IRSensorPin = 9;  

bool irPreviouslyConnected = true;
unsigned long irDisconnectedStartTime = 0;
const unsigned long irDisconnectThreshold = 2000;  // 2 seconds


// ANGLE CALC -------------------------------
float x, y, z;
float gx, gy, gz;
float dt;
float angleY = 0;
float prev_angle = 0;
float curr_angle = 0;
float accel_angle, curr_gyro_angle;
float k = 0.95;  // higher is more gyro
float weighted_angle;
int first_run = 0;
unsigned long current_time_ang = 0;
unsigned long prev_time_ang = 0;

// PID -------------------------------
unsigned long current_time_pid = 0;
unsigned long prev_time_pid = 0;
double dt_PID = 0;

double setpoint = 0;
// double Kp = 13;
// double Ki = 180;
// double Kd = 1.2;

double Kp = 8.75;
double Ki = 170;
double Kd = 0.6;

float input = 0;
float last_error = 0;
float err = 0;
float derivative = 0;
float integral = 0;
float PIDout = 0;
float PWMout = 0;

// ROBOT MOVEMENT ---------------------

unsigned long movementCommandTime = 0;
bool movementTimerActive = false;

double targetSetpoint = 0;  // Where we want to go
double rampRate = 0.25;     // How much to change the setpoint per loop

bool left_flag = false;
bool right_flag = false;

float LeftScale = 1;
float RightScale = 1;
float PWMscaledL;
float PWMscaledR;

// BLUETOOTH ---------------------
char command;

// SONAR ----------------------
const int trigPin = 12;
const int echoPin = 11;

volatile unsigned long startTime = 0;
volatile unsigned long endTime = 0;
volatile bool measurementDone = false;

float distance = 0;

float targetSetpointSONAR = 0;


// RAMP ------------

// double targetSetpointRAMP = 0;

// STARTUP

bool imuReady = false;
int imuWarmupCount = 0;
const int imuWarmupTarget = 10;  // ~10 IMU readings
bool firstPIDCycle = true;

// SERVO MOTOR ----------------

Servo myServo;

bool servoMoving = false;
unsigned long servoMoveStartTime = 0;
const unsigned long servoMoveDuration = 20;  // adjust for degree change

int servoSpeedCW = 10;  // adjust as needed (0–180; 90 = stop)
int servoSpeedCCW = 170;

// SERVO MOTOR 2 GRIPPER -----

Servo myServo2;

bool servo2Active = false;
unsigned long servo2StartTime = 0;
const unsigned long servo2Duration = 300;  // Adjust as needed
int servo2SpeedOpen = 140;                 // For opening 
int servo2SpeedClose = 40;                 // For closing 

int openPosition = 0;

int closePosition = 90;

double targetSetpointGRIP = 0;


// PARKING SEQUENCE ----------

double targetSetpointEND = 0;
bool endSequenceActive = false;
unsigned long endSequenceStartTime = 0;
const unsigned long endSequenceDuration = 500;  // milliseconds

bool servoEndSequenceActive = false;
unsigned long servoEndStartTime = 0;
const unsigned long servoEndDuration = 120;  // Duration of movement in ms (tweak as needed)
int servoEndSpeed = 10;                      // Direction of end motion

// DEPARTING SEQUENCE ---------

bool startSequenceActive = false;
unsigned long startSequenceStartTime = 0;
const unsigned long startSequenceDuration = 120;  // adjust as needed
int servoStartSpeed = 170;                        // CW or CCW depending on how you want to start

// SPEAKER SOUND

bool sonarSoundPlayed = false;
bool sonarSoundUp = true;
int trackNumber = 1;
int maxTrackNumber = 10;

// SETUP ---------------------------------------------------
void setup() {
  Serial.begin(9600);
  while (!Serial)
    ;

  pinMode(BUTTON_SEND_VALUES, INPUT_PULLUP);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }
  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1)
      ;
  }

  BLE.setLocalName("BLE-DEVICE-B9");
  BLE.setDeviceName("BLE-DEVICE-B9");

  customService.addCharacteristic(customCharacteristic);
  BLE.addService(customService);
  customCharacteristic.writeValue("Waiting for data");
  BLE.advertise();

  Serial.println("Bluetooth device active, waiting for connections...");

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(echoPin), echoISR, CHANGE);

  // SERVO
  myServo.attach(SERVO_PIN);
  myServo.write(90);  // stop at start

  myServo2.attach(SERVO2_PIN);
  myServo2.write(0);  // Start in open postion

  // SPEAKER INPUT

  Serial1.begin(9600);  // Hardware serial to DFPlayer (D1 TX, D0 RX)

  Serial.println("Initializing DFPlayer...");

  if (!myDFPlayer.begin(Serial1)) {
    Serial.println("DFPlayer Mini not found!");
    while (true)
      ;  // Stop here
  }

  Serial.println("DFPlayer Mini ready.");
  myDFPlayer.volume(16);  // Set volume (0 to 30)
  myDFPlayer.play(12);    // Play the first track
}

// USER FUNCTIONS ---------------------------------------------

void checkSerialInput() {
  if (Serial.available()) {
    String inputString = Serial.readStringUntil('\n');
    inputString.trim();

    // === Handle single-character movement commands ===
    if (inputString.length() == 1) {
      char keyboard = inputString.charAt(0);

      if (keyboard == '0') {
        targetSetpoint = 7;
        movementCommandTime = millis();
        movementTimerActive = true;
        Serial.println("Target setpoint: 1 (lean forward)");
      } else if (keyboard == 'p') {
        targetSetpoint = -7;
        movementCommandTime = millis();
        movementTimerActive = true;
        Serial.println("Target setpoint: -1 (lean backward)");
      } else if (keyboard == ';') {
        targetSetpoint = 0.0;
        movementTimerActive = false;  // cancel any running timer
        Serial.println("Setpoint reset to 0.0 (manual stop)");
      }
      // LEFT AND RIGHT MOVEMENT
      else if (keyboard == 'o') {
        targetSetpoint = 1.0;
        left_flag = true;
        LeftScale = 0.5;
        RightScale = 1.4;
        movementCommandTime = millis();
        movementTimerActive = true;
        Serial.println("Left Flag Set");
      } else if (keyboard == '[') {
        targetSetpoint = 1.0;
        right_flag = true;
        RightScale = 0.5;
        LeftScale = 1.4;
        movementCommandTime = millis();
        movementTimerActive = true;
        Serial.println("Right Flag Set");
      }

      else if (keyboard == 's') {
        if (!systemEnabled && !startSequenceActive) {
          // Start sequence begins
          startSequenceStartTime = millis();
          startSequenceActive = true;

          myServo.write(servoStartSpeed);
          Serial.println("Startup sequence initiated... (with servo)");
        }
      }


      else if (keyboard == 'x') {
        if (systemEnabled && !endSequenceActive) {
          targetSetpointEND = 2.0;  // Graceful lean forward
          endSequenceStartTime = millis();
          endSequenceActive = true;

          // Trigger end sequence servo nudge
          myServo.write(servoEndSpeed);
          servoEndStartTime = millis();
          servoEndSequenceActive = true;

          Serial.println("Graceful shutdown initiated... (with servo)");
        } else {
          resetSystem();
          Serial.println("System already disabled or end sequence not active — force reset.");
        }
      }


      else if (keyboard == 'u') {  // Nudge clockwise
        if (!servoMoving) {
          myServo.write(servoSpeedCW);
          servoMoveStartTime = millis();
          servoMoving = true;
          Serial.println("Servo nudging CW");
        }
      } else if (keyboard == 'j') {  // Nudge counter-clockwise
        if (!servoMoving) {
          myServo.write(servoSpeedCCW);
          servoMoveStartTime = millis();
          servoMoving = true;
          Serial.println("Servo nudging CCW");
        }
      }

      else {
        Serial.println("Unknown command.");
      }
    }

    // === Handle PID tuning input like "12,180,1.2" ===
    else {
      int firstComma = inputString.indexOf(',');
      int secondComma = inputString.indexOf(',', firstComma + 1);

      if (firstComma > 0 && secondComma > firstComma) {
        String kpStr = inputString.substring(0, firstComma);
        String kiStr = inputString.substring(firstComma + 1, secondComma);
        String kdStr = inputString.substring(secondComma + 1);

        Kp = kpStr.toFloat();
        Ki = kiStr.toFloat();
        Kd = kdStr.toFloat();

        integral = 0;
        last_error = 0;

        Serial.print("Updated PID values: ");
        Serial.print("Kp = ");
        Serial.print(Kp);
        Serial.print(", Ki = ");
        Serial.print(Ki);
        Serial.print(", Kd = ");
        Serial.println(Kd);
      } else {
        Serial.println("Invalid input. Use Kp,Ki,Kd or a single letter (u/d/s).");
      }
    }
  }
}



void updateSetpointRamp() {
  double diff = (targetSetpoint + targetSetpointSONAR + targetSetpointEND + targetSetpointGRIP) - setpoint;
  if (abs(diff) > rampRate) {
    setpoint += rampRate * ((diff > 0) ? 1 : -1);
  } else {
    setpoint = (targetSetpoint + targetSetpointSONAR + targetSetpointEND + targetSetpointGRIP);  // Close enough, snap to final value
  }
}

void PIDCalc() {
  current_time_pid = micros();
  dt_PID = (current_time_pid - prev_time_pid) / 1000000.0;
  prev_time_pid = current_time_pid;

  if (dt_PID < 0.001) return;  // Skip if time is too small

  input = weighted_angle;
  err = setpoint - input;

  integral += Ki * err * dt_PID;
  integral = constrain(integral, -200, 200);

  if (firstPIDCycle) {
    derivative = 0;
    firstPIDCycle = false;
  } else {
    derivative = Kd * ((err - last_error) / dt_PID);
  }

  PIDout = (Kp * err) + integral + derivative;
  PWMout = constrain(PIDout, -255, 255);

  last_error = err;
}


void AngleCalc() {
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    angleY = atan(y / z) * 180 / PI;
    accel_angle = angleY;

    if (first_run == 0) {
      prev_angle = accel_angle;
      first_run += 1;
    }

    IMU.readGyroscope(gx, gy, gz);

    current_time_ang = micros();
    dt = (current_time_ang - prev_time_ang) / 1000000.0;
    prev_time_ang = current_time_ang;

    weighted_angle += (-gx * dt);
    weighted_angle = (k)*weighted_angle + (1.0 - k) * accel_angle;
  }
}

void bluetooth() {
  BLEDevice central = BLE.central();

  if (central) {
    //Serial.print("Connected to central: ");
    //Serial.println(central.address());
    //digitalWrite(LED_BUILTIN, HIGH);


    if (customCharacteristic.written()) {
      int length = customCharacteristic.valueLength();
      const unsigned char* receivedData = customCharacteristic.value();

      if (length >= 0) {  // Expecting exactly 3 bytes (Kp, Ki, Kd)
        // Kp = float(receivedData[0] + (receivedData[1] * 256)) / 100;  // Scale values down
        // //Serial.print(receivedData[2]);
        // Ki = float(receivedData[2] + (receivedData[3] * 256)) / 100;
        // Kd = float(receivedData[4] + (receivedData[5] * 256)) / 100;
        // Serial.println(Kp);
        // Serial.println(Ki);
        // Serial.println(Kd);
        command = (char)receivedData[0];
        Serial.println(command);


        // NEED TO MOVE THIS TO BLUETOOTH UPDATE FUNCTION
        if (command == '1') {
          targetSetpoint = 5.5;
          Serial.println("Target setpoint: BLE (lean forward)");
        } else if (command == '2') {
          targetSetpoint = -5.5;
          Serial.println("Target setpoint: -BLE (lean backward)");
        } else if (command == '3') {  // LEFT
          left_flag = true;
          LeftScale = 0.4;
          RightScale = 1;
          Serial.println("Turning LEFT BLE");
        } else if (command == '4') {  // RIGHT
          right_flag = true;
          RightScale = 0.4;
          LeftScale = 1;
          Serial.println("Turning RIGHT BLE");
        } else if (command == '0') {  // NEUTRAL POSITION
          targetSetpoint = 0;
          right_flag = false;
          left_flag = false;
          RightScale = 1;
          LeftScale = 1;
          Serial.println("NEUTRAL - TARGET SETPOINT: 0");
        }

        else if (command == '5' && !systemEnabled) {
          if (!systemEnabled && !startSequenceActive) {
            // Start sequence begins
            startSequenceStartTime = millis();
            startSequenceActive = true;

            myServo.write(servoStartSpeed);
            Serial.println("Startup sequence initiated... (with servo)");
          }
        }

        else if (command == '5' && systemEnabled) {
          if (systemEnabled && !endSequenceActive) {
            targetSetpointEND = 2;  // Graceful lean forward
            endSequenceStartTime = millis();
            endSequenceActive = true;

            // Trigger end sequence servo nudge
            myServo.write(servoEndSpeed);
            servoEndStartTime = millis();
            servoEndSequenceActive = true;


            Serial.println("Graceful shutdown initiated... (with servo)");
          } else {
            resetSystem();
            Serial.println("System already disabled or end sequence not active — force reset.");
          }
        }

        else if (command == '6') {

          Serial.println("Received 6");
        } else if (command == '7') {

          myServo2.write(openPosition);
          targetSetpointGRIP = 0;
          Serial.println("Servo 2 OPEN");

        } else if (command == '8') {

          myServo2.write(closePosition);
          targetSetpointGRIP = 0.6;
          Serial.println("Servo 2 CLOSE");
        }

        else if (command == '9' && sonarSoundUp) {
          myDFPlayer.volume(0);
          sonarSoundUp = false;

        } else if (command == '9' && !sonarSoundUp) {
          myDFPlayer.volume(22);
          sonarSoundUp = true;
        }


        else if (command == 'b') {
          if (trackNumber >= maxTrackNumber) {
            trackNumber = 1;
          } else {
            trackNumber += 1;
          }
          myDFPlayer.play(trackNumber);
          Serial.println("command 'b' recieved");
          Serial.print("Track #: ");
          Serial.println(trackNumber);


        } else if (command == 'a') {
          if (trackNumber <= 1) {
            trackNumber = maxTrackNumber;
          } else {
            trackNumber -= 1;
          }
          myDFPlayer.play(trackNumber);
          Serial.println("command 'a' recieved");
          Serial.print("Track #: ");
          Serial.println(trackNumber);
        }

        // else if (command == '9') {
        //   targetSetpointRAMP += 5;
        //   Serial.println("Target setpoint: -1.2 (lean backward)");
        // } else if (command == '10') {
        //   targetSetpointRAMP -= 5;
        //   Serial.println("Target setpoint: -1.2 (lean backward)");
        // }




      } else {
        Serial.println("Invalid data received.");
      }
    }


    // digitalWrite(LED_BUILTIN, LOW);
    // Serial.println("Disconnected from central.");
  }
}

void DriveMotors() {

  // // FAST DECAY
  // if (PWMout >= 0) {
  //   analogWrite(BIN1, PWMout);
  //   analogWrite(AIN1, PWMout);
  //   analogWrite(BIN2, LOW);
  //   analogWrite(AIN2, LOW);
  // }

  // if (PWMout < 0) {
  //   analogWrite(BIN1, LOW);
  //   analogWrite(AIN1, LOW);
  //   analogWrite(BIN2, abs(PWMout));
  //   analogWrite(AIN2, abs(PWMout));
  // }

  if (left_flag || right_flag) {

    if (PWMout >= 0) {

      if ((PWMout * LeftScale) > 255) {
        PWMscaledL = 255;
      } else {
        PWMscaledL = (PWMout * LeftScale);
      }

      if ((PWMout * RightScale) > 255) {
        PWMscaledR = 255;
      } else {
        PWMscaledR = (PWMout * RightScale);
      }
      analogWrite(BIN1, 255);
      analogWrite(AIN1, 255);
      analogWrite(BIN2, 255 - PWMscaledL);
      analogWrite(AIN2, 255 - PWMscaledR);
    }

    if (PWMout < 0) {

      if (abs((PWMout * LeftScale)) > 255) {
        PWMscaledL = 255;
      } else {
        PWMscaledL = abs((PWMout * LeftScale));
      }

      if (abs((PWMout * RightScale)) > 255) {
        PWMscaledR = 255;
      } else {
        PWMscaledR = abs((PWMout * RightScale));
      }
      analogWrite(BIN1, 255 - PWMscaledL);
      analogWrite(AIN1, 255 - PWMscaledR);
      analogWrite(BIN2, 255);
      analogWrite(AIN2, 255);
    }
  }

  else {

    if (PWMout >= 0) {
      analogWrite(BIN1, 255);
      analogWrite(AIN1, 255);
      analogWrite(BIN2, 255 - PWMout);
      analogWrite(AIN2, 255 - PWMout);
    }

    if (PWMout < 0) {
      analogWrite(BIN1, 255 - abs(PWMout));
      analogWrite(AIN1, 255 - abs(PWMout));
      analogWrite(BIN2, 255);
      analogWrite(AIN2, 255);
    }
  }
}

void triggerUltrasonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
}

void echoISR() {
  if (digitalRead(echoPin) == HIGH) {
    startTime = micros();
  } else {
    endTime = micros();
    measurementDone = true;
  }
}

void WallDetection() {
  const int minVal = 15;  // sonar reading where output should be maxOutput
  const int maxVal = 35;  // sonar reading where output should be 0
  const float maxOutput = 6;

  if (distance >= maxVal) {
    targetSetpointSONAR = 0;
    sonarSoundPlayed = false;  // Reset trigger so we can play sound again later
  } else {
    if (distance <= minVal) {
      targetSetpointSONAR = -maxOutput;
    } else {
      float t = (float)(maxVal - distance) / (maxVal - minVal);  // t goes from 0 to 1
      targetSetpointSONAR = -maxOutput * t * t;
    }

    // Sound trigger
    if (!sonarSoundPlayed && distance < 30) {
      myDFPlayer.volume(30);
      myDFPlayer.play(11);
      sonarSoundPlayed = true;
      Serial.println("Sonar triggered sound below 30cm");
    }
  }
}


void sonarMeasure() {
  static unsigned long lastTriggerTime = 0;
  unsigned long currentTime = micros();

  // Trigger every 60ms (safe interval to avoid echo overlap)
  if (currentTime - lastTriggerTime >= 60000) {
    triggerUltrasonic();
    lastTriggerTime = currentTime;
  }

  if (measurementDone) {
    unsigned long duration = endTime - startTime;
    distance = (duration * 0.0343) / 2.0;
    measurementDone = false;
  }
}


void resetSystem() {
  // PID
  integral = 0;
  last_error = 0;
  err = 0;
  derivative = 0;
  PIDout = 0;
  PWMout = 0;

  // Angle
  weighted_angle = 0;
  prev_angle = 0;
  first_run = 0;

  // Movement
  setpoint = 0;
  targetSetpoint = 0;
  left_flag = false;
  right_flag = false;
  LeftScale = 1;
  RightScale = 1;
  movementTimerActive = false;

  // IMU warm-up reset
  imuReady = false;
  imuWarmupCount = 0;
  firstPIDCycle = true;

  targetSetpointEND = 0;

  DriveMotors();

  Serial.println("System reset complete.");
}


void checkIRSensor() {
  bool irConnected = digitalRead(IRSensorPin) == HIGH;

  if (!irConnected) {
    if (irPreviouslyConnected) {
      irDisconnectedStartTime = millis();  // just started disconnecting
      // Serial.println("IR NOT connected");
    }

    if (millis() - irDisconnectedStartTime >= irDisconnectThreshold) {
      if (systemEnabled && !endSequenceActive) {
        targetSetpointEND = 2;  // Graceful lean forward
        endSequenceStartTime = millis();
        endSequenceActive = true;

        // Trigger end sequence servo nudge
        myServo.write(servoEndSpeed);
        servoEndStartTime = millis();
        servoEndSequenceActive = true;

        Serial.println("IR disconnected >2s — graceful shutdown initiated.");
      }
    }

    irPreviouslyConnected = false;
  } else {
    irPreviouslyConnected = true;
    // Serial.println("IR connected");
  }
}


void handleTimedEvents() {

  // TURNING CONTROL (keyboard)
  if (movementTimerActive && millis() - movementCommandTime >= 350) {
    targetSetpoint = 0.0;
    left_flag = false;
    right_flag = false;
    movementTimerActive = false;
    LeftScale = 1;
    RightScale = 1;
    Serial.println("Movement duration done — returning to 0.0");
  }

  // PARKING STAND STARTING/END (MANUAL)
  if (servoMoving && millis() - servoMoveStartTime >= servoMoveDuration) {
    myServo.write(90);  // Stop the servo
    servoMoving = false;
    Serial.println("Servo movement done, stopped.");
  }

  // LEAN BEFORE PARKING
  if (endSequenceActive && millis() - endSequenceStartTime >= endSequenceDuration) {
    systemEnabled = false;
    endSequenceActive = false;
    resetSystem();
    Serial.println("System DISABLED after graceful stop.");
  }

  // PARKING STAND ENDING
  if (servoEndSequenceActive && millis() - servoEndStartTime >= servoEndDuration) {
    myServo.write(90);  // Stop servo
    servoEndSequenceActive = false;
    Serial.println("Servo end-sequence movement complete.");
  }

  if (servo2Active && millis() - servo2StartTime >= servo2Duration) {
    myServo2.write(90);  // Stop movement
    servo2Active = false;
    Serial.println("Servo 2 stopped");
  }
}

void startup() {
  if (startSequenceActive && millis() - startSequenceStartTime >= startSequenceDuration) {
    myServo.write(90);  // Stop the servo
    startSequenceActive = false;
    systemEnabled = true;

    // === RESET TIMING TO AVOID LARGE dt ON FIRST CYCLE ===
    prev_time_pid = micros();
    prev_time_ang = micros();

    firstPIDCycle = true;  // derivative = 0 on first PID

    Serial.println("Startup complete. System ENABLED.");
  }
}

bool imuWarmup() {
  if (!imuReady) {
    imuWarmupCount++;
    if (imuWarmupCount >= imuWarmupTarget) {
      imuReady = true;
      Serial.println("IMU warmed up — starting PID");
    } else {
      return false;  // Not ready yet, skip rest of loop
    }
  }
  return true;  // IMU ready
}


// MAIN LOOP ------------------------------------------------------
void loop() {
  if (!systemEnabled) {  // PARKED/OFF STATE
    checkSerialInput();  // FOR COMPUTER KEYBOARD DEBUG / CONTROL
    bluetooth();         // READ BLUETOOTH VALUES
  }

  startup();

  if (systemEnabled) {  // BALANCING STATE

    checkIRSensor();  // CHECK IR SENSOR SIGNAL FOR PARKING MODE
    sonarMeasure();   // MEASURE DISTANCE
    WallDetection();  // AVOID WALLS/OBJECTS BASED ON DISTANCE

    checkSerialInput();  // OPTIONAL COMPUTER KEYBOARD DEBUG / CONTROL
    bluetooth();         // READ BLUETOOTH VALUES

    updateSetpointRamp();  // SMOOTH TRANSITION TO NEW SETPOINT ANGLE
    AngleCalc();           // ANGLE MEASUREMENT

    if (!imuWarmup()) {  // ALLOW IMU A FEW CYCLES BEFORE RUNNING PID
      return;            // IMU not ready — skip PID and motors
    }

    PIDCalc();            // CONTROL LOOP
    DriveMotors();        // OUTPUT SIGNALS TO MOTOR DRIVERS
    handleTimedEvents();  // EVENT TIMERS (NON-BLOCKING)
  }
}
