#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ---------- nRF24 Setup ----------
RF24 radio(9, 10); // CE, CSN pins
const byte txAddress[6] = "1Node1"; // Rover → Remote
const byte rxAddress[6] = "2Node2"; // Rover ← Remote

// ---------- Servo Driver ----------
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ---------- Motors ----------
const int motorLeft1 = 2;
const int motorLeft2 = 3;
const int motorRight1 = 4;
const int motorRight2 = 5;

// ---------- Sensors ----------
const int dirPin = 6;
const int trigPin = 7;
const int echoPin = 8;

// ---------- Arm Servo Angles ----------
int arm1Angle = 90;
int arm2Angle = 90;
int arm3Angle = 90;

// ---------- Command Struct ----------
struct RemoteCommand {
  int analogMove;   // -100 to 100
  int analogTurn;   // -100 to 100
  int headX;        // 0-1023
  int headY;        // 0-1023
  bool armUp;
  bool armDown;
  bool armLeft;
  bool armRight;
  bool armUp2;
  bool armDown2;
  bool flash;
};

struct SensorData {
  int distance;
  bool motionDetected;
};

RemoteCommand command;
SensorData sensors;

void setup() {
  Serial.begin(9600);

  // Motors
  pinMode(motorLeft1, OUTPUT);
  pinMode(motorLeft2, OUTPUT);
  pinMode(motorRight1, OUTPUT);
  pinMode(motorRight2, OUTPUT);

  // Sensors
  pinMode(dirPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Servo driver
  pwm.begin();
  pwm.setPWMFreq(60); // Servo frequency

  // nRF24
  radio.begin();
  radio.openWritingPipe(txAddress);
  radio.openReadingPipe(1, rxAddress);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
}

void loop() {
  receiveRemoteCommand();
  readSensors();
  controlMotors();
  moveArmAndHead();
  sendSensorData();
  delay(50); // Smooth control
}

// ---------- Functions ---------- //
void readSensors() {
  sensors.motionDetected = digitalRead(dirPin);

  // Ultrasonic
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 20000);
  sensors.distance = duration * 0.034 / 2;
}

void receiveRemoteCommand() {
  if (radio.available()) {
    radio.read(&command, sizeof(RemoteCommand));
  }
}

void controlMotors() {
  int move = command.analogMove; // -100..100
  int turn = command.analogTurn; // -100..100

  // TANK MIXING
  int left = constrain(move + turn, -100, 100);
  int right = constrain(move - turn, -100, 100);

  // LEFT MOTOR
  if (left > 0) {
    digitalWrite(motorLeft1, HIGH);
    digitalWrite(motorLeft2, LOW);
  } else if (left < 0) {
    digitalWrite(motorLeft1, LOW);
    digitalWrite(motorLeft2, HIGH);
  } else {
    digitalWrite(motorLeft1, LOW);
    digitalWrite(motorLeft2, LOW);
  }

  // RIGHT MOTOR
  if (right > 0) {
    digitalWrite(motorRight1, HIGH);
    digitalWrite(motorRight2, LOW);
  } else if (right < 0) {
    digitalWrite(motorRight1, LOW);
    digitalWrite(motorRight2, HIGH);
  } else {
    digitalWrite(motorRight1, LOW);
    digitalWrite(motorRight2, LOW);
  }
}

void moveArmAndHead() {
  // Head servos
  pwm.setPWM(0, 0, map(command.headX, 0, 1023, 150, 600));
  pwm.setPWM(1, 0, map(command.headY, 0, 1023, 150, 600));

  // Arm servos
  if (command.armUp) arm1Angle = min(arm1Angle + 5, 180);
  if (command.armDown) arm1Angle = max(arm1Angle - 5, 0);
  if (command.armLeft) arm2Angle = min(arm2Angle + 5, 180);
  if (command.armRight) arm2Angle = max(arm2Angle - 5, 0);
  if (command.armUp2) arm3Angle = min(arm3Angle + 5, 180);
  if (command.armDown2) arm3Angle = max(arm3Angle - 5, 0);

  pwm.setPWM(2, 0, map(arm1Angle, 0, 180, 150, 600));
  pwm.setPWM(3, 0, map(arm2Angle, 0, 180, 150, 600));
  pwm.setPWM(4, 0, 180, 150, 600);
}

void sendSensorData() {
  radio.stopListening();
  radio.write(&sensors, sizeof(SensorData));
  radio.startListening();
}
