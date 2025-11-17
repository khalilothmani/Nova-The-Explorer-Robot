#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// LCD (I2C)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// nRF24
RF24 radio(9, 10);
const byte txAddress[6] = "2Node2"; 
const byte rxAddress[6] = "1Node1";

// Joysticks
const int analogMovePin = A0;
const int analogTurnPin = A1;
const int headXPin = A2;
const int headYPin = A3;

// Buttons
const int armUpPin = 2;
const int armDownPin = 3;
const int armLeftPin = 4;
const int armRightPin = 5;
const int armUp2Pin = 6;
const int armDown2Pin = 7;
const int flashPin = 8;

struct RemoteCommand {
  int analogMove;
  int analogTurn;
  int headX;
  int headY;
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

  lcd.init();
  lcd.backlight();
  lcd.print("Controller Ready");

  // Buttons
  pinMode(armUpPin, INPUT_PULLUP);
  pinMode(armDownPin, INPUT_PULLUP);
  pinMode(armLeftPin, INPUT_PULLUP);
  pinMode(armRightPin, INPUT_PULLUP);
  pinMode(armUp2Pin, INPUT_PULLUP);
  pinMode(armDown2Pin, INPUT_PULLUP);
  pinMode(flashPin, INPUT_PULLUP);

  radio.begin();
  radio.openWritingPipe(txAddress);
  radio.openReadingPipe(0, rxAddress);
  radio.setPALevel(RF24_PA_LOW);
  radio.stopListening();
}

void loop() {
  readInputs();
  sendCommand();
  receiveSensorData();
  displayLCD();
  delay(50);
}

// ===== INPUTS =====
void readInputs() {
  command.analogMove = map(analogRead(analogMovePin), 0, 1023, -100, 100);
  command.analogTurn = map(analogRead(analogTurnPin), 0, 1023, -100, 100);

  command.headX = analogRead(headXPin);
  command.headY = analogRead(headYPin);

  command.armUp = !digitalRead(armUpPin);
  command.armDown = !digitalRead(armDownPin);
  command.armLeft = !digitalRead(armLeftPin);
  command.armRight = !digitalRead(armRightPin);
  command.armUp2 = !digitalRead(armUp2Pin);
  command.armDown2 = !digitalRead(armDown2Pin);
  command.flash = !digitalRead(flashPin);
}

// ===== SEND COMMAND =====
void sendCommand() {
  radio.stopListening();
  radio.write(&command, sizeof(RemoteCommand));
}

// ===== RECEIVE SENSOR DATA =====
void receiveSensorData() {
  radio.startListening();
  if (radio.available()) {
    radio.read(&sensors, sizeof(SensorData));
  }
  radio.stopListening();
}

// ===== LCD =====
void displayLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Dist: "); lcd.print(sensors.distance); lcd.print("cm");

  lcd.setCursor(0, 1);
  lcd.print("Motion: ");
  lcd.print(sensors.motionDetected ? "YES" : "NO ");
}
