/*--
**********************************************************
****************************-
-- Produced By: Computer Systems Engineering @ UoN
-- Author: Parastoo Shojaei Sarcheshmeh
-- Project Title: A Smart IR-Controlled Autonomous Vehicle Using Arduino Uno
-- Modifications:  Function-based implementation
--
**********************************************************
*************************** --*/

#include <LiquidCrystal.h>
#include <IRremote.hpp>

// LCD wiring: RS=D13, E=D12, D4=D8, D5=D4, D6=D3, D7=D2
LiquidCrystal lcd(13, 12, 8, 4, 3, 2);

// IR receiver pin
#define IR_RECEIVE_PIN 11

// Ultrasonic pin
const int PING_PIN = 7;

// Push button pin (mode switch)
const int BTN_PIN = A0;

// Motor control pins (L293D)
const int M1_F = 6;
const int M1_B = 5;
const int M2_F = 10;
const int M2_B = 9;

// IR command codes
#define CMD_STOP  0x0C
#define CMD_FWD   0x10
#define CMD_BACK  0x11
#define CMD_LEFT  0x12
#define CMD_RIGHT 0x14

// System modes
enum Mode { MANUAL, AUTO };
Mode mode = MANUAL;

// Timing for sensor and LCD refresh
unsigned long lastSense = 0;
const unsigned long SENSE_PERIOD_MS = 200;

unsigned long lastLCD = 0;
const unsigned long LCD_PERIOD_MS = 200;

// Button debounce variables
bool lastBtnRead = HIGH;
bool btnStable = HIGH;
unsigned long lastDebounce = 0;
const unsigned long DEBOUNCE_MS = 60;

// Runtime state for LCD
const char* dirText = "STOP";
float lastCm = -1;

// Manual mode safety distances
const float STOP_CM    = 25.0;
const float RELEASE_CM = 30.0;
bool safetyLock = false;

// Auto mode distances and timings
const float AUTO_TRIGGER_CM = 25.0;
const float RELEASE_AUTO_CM = 35.0;

const unsigned long AUTO_STOP_MS  = 2000;
const unsigned long AUTO_RIGHT_MS = 1200;
const unsigned long AUTO_FWD_MS   = 1200;

bool avoidArmed = true;

// Auto FSM states
enum AutoStep { AUTO_FORWARD, AUTO_STOPPING, AUTO_TURN_RIGHT, AUTO_FORCE_FORWARD };
AutoStep autoStep = AUTO_FORWARD;
unsigned long autoT0 = 0;

// Motor control function
void moveMotors(int m1f, int m1b, int m2f, int m2b) {
  digitalWrite(M1_F, m1f);
  digitalWrite(M1_B, m1b);
  digitalWrite(M2_F, m2f);
  digitalWrite(M2_B, m2b);
}

// Stop both motors
void stopMotors() {
  moveMotors(0,0,0,0);
}

// Button debounce
bool buttonPressedEvent() {
  bool reading = digitalRead(BTN_PIN);

  if (reading != lastBtnRead) {
    lastDebounce = millis();
    lastBtnRead = reading;
  }

  if (millis() - lastDebounce > DEBOUNCE_MS) {
    if (reading != btnStable) {
      btnStable = reading;
      if (btnStable == LOW) return true; // INPUT_PULLUP => pressed is LOW
    }
  }
  return false;
}

// Ultrasonic raw pulse read
long readPingMicroseconds_RAW() {
  pinMode(PING_PIN, OUTPUT);
  digitalWrite(PING_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(PING_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(PING_PIN, LOW);

  pinMode(PING_PIN, INPUT);
  return pulseIn(PING_PIN, HIGH, 30000);
}

// Ultrasonic stable read
long readPingMicroseconds_STABLE() {
  IrReceiver.stop();
  delayMicroseconds(200);
  long us = readPingMicroseconds_RAW();
  IrReceiver.start();
  return us;
}

// Update distance in cm
void updateDistance() {
  long us = readPingMicroseconds_STABLE();
  if (us == 0) lastCm = -1;
  else lastCm = us / 58.0; // convert microseconds to cm
}

// LCD line 0 (mode + direction)
void lcdLine0() {
  lcd.setCursor(0, 0);
  if (mode == MANUAL) lcd.print("MODE:MAN ");
  else               lcd.print("MODE:AUTO");
  lcd.print(" ");
  lcd.print(dirText);
  lcd.print("     ");
}

// LCD line 1 (distance)
void lcdLine1() {
  lcd.setCursor(0, 1);
  lcd.print("Dist:");
  if (lastCm < 0) lcd.print("---");
  else lcd.print(lastCm, 1);
  lcd.print("cm   ");
}

// Manual mode safety lock
void applySafetyManualOnly() {
  if (lastCm < 0) return;

  if (!safetyLock && lastCm <= STOP_CM) {
    safetyLock = true;
    stopMotors();
    dirText = "STOP";
  }
  if (safetyLock && lastCm >= RELEASE_CM) {
    safetyLock = false;
  }
}

// Manual mode IR control
void handleManualIR() {
  if (!IrReceiver.decode()) return;
  byte cmd = IrReceiver.decodedIRData.command;
  IrReceiver.resume();

  // block forward if safety lock is active
  if (safetyLock && cmd == CMD_FWD) {
    stopMotors();
    dirText = "STOP";
    return;
  }

  if (cmd == CMD_FWD) { moveMotors(1,0,1,0); dirText = "FORWARD"; }
  else if (cmd == CMD_BACK) { moveMotors(0,1,0,1); dirText = "BACKWARD"; }
  else if (cmd == CMD_LEFT) { moveMotors(1,0,0,1); dirText = "LEFT"; }
  else if (cmd == CMD_RIGHT) { moveMotors(0,1,1,0); dirText = "RIGHT"; }
  else if (cmd == CMD_STOP) { stopMotors(); dirText = "STOP"; }
}

// Reset auto FSM
void autoReset() {
  autoStep = AUTO_FORWARD;
  autoT0 = millis();
  avoidArmed = true;
}

// Auto mode logic
void handleAutoSimple() {
  if (mode != AUTO) return;

  unsigned long now = millis();

  // if distance is invalid, stop
  if (lastCm < 0) {
    stopMotors();
    dirText = "STOP";
    return;
  }

  // re-arm avoidance after moving away
  if (!avoidArmed && lastCm >= RELEASE_AUTO_CM) {
    avoidArmed = true;
  }

  // trigger avoidance when close
  if (autoStep == AUTO_FORWARD && avoidArmed && lastCm <= AUTO_TRIGGER_CM) {
    autoStep = AUTO_STOPPING;
    autoT0 = now;
    avoidArmed = false;
  }

  switch (autoStep) {
    case AUTO_FORWARD:
      moveMotors(1,0,1,0);
      dirText = "FORWARD";
      break;

    case AUTO_STOPPING:
      stopMotors();
      dirText = "STOP";
      if (now - autoT0 >= AUTO_STOP_MS) {
        autoStep = AUTO_TURN_RIGHT;
        autoT0 = now;
      }
      break;

    case AUTO_TURN_RIGHT:
      moveMotors(0,1,1,0);
      dirText = "RIGHT";
      if (now - autoT0 >= AUTO_RIGHT_MS) {
        autoStep = AUTO_FORCE_FORWARD;
        autoT0 = now;
      }
      break;

    case AUTO_FORCE_FORWARD:
      moveMotors(1,0,1,0);
      dirText = "FORWARD";
      if (now - autoT0 >= AUTO_FWD_MS) {
        autoStep = AUTO_FORWARD;
        autoT0 = now;
      }
      break;
  }
}

// Setup
void setup() {
  Serial.begin(9600);

  pinMode(M1_F, OUTPUT); pinMode(M1_B, OUTPUT);
  pinMode(M2_F, OUTPUT); pinMode(M2_B, OUTPUT);
  stopMotors();

  pinMode(BTN_PIN, INPUT_PULLUP);

  IrReceiver.begin(IR_RECEIVE_PIN, DISABLE_LED_FEEDBACK);

  lcd.begin(16, 2);
  lcd.setCursor(0, 0); lcd.print("MODE:MAN STOP   ");
  lcd.setCursor(0, 1); lcd.print("Dist:---cm      ");

  autoReset();
}

// Main loop
void loop() {

  // Mode toggle
  if (buttonPressedEvent()) {
    mode = (mode == MANUAL) ? AUTO : MANUAL;

    stopMotors();
    dirText = "STOP";

    safetyLock = false;
    autoReset();
  }

  // Distance update every 200ms
  unsigned long now = millis();
  if (now - lastSense >= SENSE_PERIOD_MS) {
    lastSense = now;
    updateDistance();
  }

  // Run current mode
  if (mode == MANUAL) {
    applySafetyManualOnly();
    handleManualIR();
    applySafetyManualOnly();
  } else {
    handleAutoSimple();
  }

  // LCD refresh
  if (now - lastLCD >= LCD_PERIOD_MS) {
    lastLCD = now;
    lcdLine0();
    lcdLine1();
  }
}
