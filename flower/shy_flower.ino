// Shy Flower
// Opens and plays Pachelbel's Canon when it's calm. Closes if someone approaches
// too fast. Stays closed >= 5 s. Reopens when the person has backed off,
// or if there's lots of light.
// Button on pin 5 = "the end": melody stops, flower slowly closes and sleeps.
// Press again to wake it up.

#include <Servo.h>

// ---------------- Pins ----------------
const int SERVO_PIN  = 2;
const int BUZZER_PIN = 8;
const int LIGHT_PIN  = A2;
const int TRIG_PIN   = 3;
const int ECHO_PIN   = 4;
const int BUTTON_PIN = 5;   // wired to GND when pressed (INPUT_PULLUP)

// ---------------- Tuning ----------------
// Servo
const int CLOSED_POS = 30;
const int OPEN_POS   = 190;
const int OPEN_STEP_MS  = 40;   // ms per degree while opening (bigger = slower)
const int CLOSE_STEP_MS = 25;   // ms per degree while closing

// Shyness
const unsigned long MIN_CLOSED_MS = 5000;  // stay closed at least this long
const float FAST_APPROACH_CM_S = 50.0;     // approaching faster than this = scary
const float SCARE_RANGE_CM     = 150.0;    // only react to things within this range
const float SAFE_DISTANCE_CM   = 60.0;     // backed away at least this far = ok to reopen
const float CALM_SPEED_CM_S    = 30.0;     // must be moving slower than this to reopen

// Light (ESP32 ADC is 0-4095). Flip the comparison in isBright() if your
// photoresistor reads LOWER when it's bright.
const int LIGHT_THRESHOLD = 100;

// Ultrasonic
const unsigned long ECHO_TIMEOUT_US = 25000;
const float MAX_CM = 300.0;
const unsigned long DIST_INTERVAL_MS = 100;  // read distance every 100 ms

// Melody
const int tempo = 150;
const int octaveMultiplier = 2;     // 1 = original pitch, 2 = up one octave
const float articulation = 0.7;     // fraction of each note actually sounded
const bool LOOP_MELODY = true;      // replay after a pause
const unsigned long LOOP_PAUSE_MS = 3000;

// ---------------- Notes (only the ones this song uses) ----------------
#define NOTE_D3  147
#define NOTE_E3  165
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_A3  220
#define NOTE_B3  247
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_A4  440

int melody[] = {
  // Cannon in D - Pachelbel
  NOTE_FS4,2, NOTE_E4,2,
  NOTE_D4,2, NOTE_CS4,2,
  NOTE_B3,2, NOTE_A3,2,
  NOTE_B3,2, NOTE_CS4,2,
  NOTE_FS4,2, NOTE_E4,2,
  NOTE_D4,2, NOTE_CS4,2,
  NOTE_B3,2, NOTE_A3,2,
  NOTE_B3,2, NOTE_CS4,2,
  NOTE_D4,2, NOTE_CS4,2,
  NOTE_B3,2, NOTE_A3,2,
  NOTE_G3,2, NOTE_FS3,2,
  NOTE_G3,2, NOTE_A3,2,

  NOTE_D4,4, NOTE_FS4,8, NOTE_G4,8, NOTE_A4,4, NOTE_FS4,8, NOTE_G4,8,
  NOTE_A4,4, NOTE_B3,8, NOTE_CS4,8, NOTE_D4,8, NOTE_E4,8, NOTE_FS4,8, NOTE_G4,8,
  NOTE_FS4,4, NOTE_D4,8, NOTE_E4,8, NOTE_FS4,4, NOTE_FS3,8, NOTE_G3,8,
  NOTE_A3,8, NOTE_G3,8, NOTE_FS3,8, NOTE_G3,8, NOTE_A3,2,
  NOTE_G3,4, NOTE_B3,8, NOTE_A3,8, NOTE_G3,4, NOTE_FS3,8, NOTE_E3,8,
  NOTE_FS3,4, NOTE_D3,8, NOTE_E3,8, NOTE_FS3,8, NOTE_G3,8, NOTE_A3,8, NOTE_B3,8,

  NOTE_G3,4, NOTE_B3,8, NOTE_A3,8, NOTE_B3,4, NOTE_CS4,8, NOTE_D4,8,
  NOTE_A3,8, NOTE_B3,8, NOTE_CS4,8, NOTE_D4,8, NOTE_E4,8, NOTE_FS4,8, NOTE_G4,8, NOTE_A4,2,
  NOTE_A4,4, NOTE_FS4,8, NOTE_G4,8, NOTE_A4,4,
  NOTE_FS4,8, NOTE_G4,8, NOTE_A4,8, NOTE_A3,8, NOTE_B3,8, NOTE_CS4,8,
  NOTE_D4,8, NOTE_E4,8, NOTE_FS4,8, NOTE_G4,8, NOTE_FS4,4, NOTE_D4,8, NOTE_E4,8,
  NOTE_FS4,8, NOTE_CS4,8, NOTE_A3,8, NOTE_A3,8,

  NOTE_CS4,4, NOTE_B3,4, NOTE_D4,8, NOTE_CS4,8, NOTE_B3,4,
  NOTE_A3,8, NOTE_G3,8, NOTE_A3,4, NOTE_D3,8, NOTE_E3,8, NOTE_FS3,8, NOTE_G3,8,
  NOTE_A3,8, NOTE_B3,4, NOTE_G3,4, NOTE_B3,8, NOTE_A3,8, NOTE_B3,4,
  NOTE_CS4,8, NOTE_D4,8, NOTE_A3,8, NOTE_B3,8, NOTE_CS4,8, NOTE_D4,8, NOTE_E4,8,
  NOTE_FS4,8, NOTE_G4,8, NOTE_A4,2,
};

const int notes = sizeof(melody) / sizeof(melody[0]) / 2;
const int wholenote = (60000 * 4) / tempo;

// ---------------- State ----------------
enum State { AWAKE, HIDING, SLEEPING };
State state = AWAKE;

Servo flower;
int servoPos = CLOSED_POS;
int servoTarget = CLOSED_POS;
unsigned long lastServoStep = 0;

// melody playback
bool melodyPlaying = false;
bool noteSounding = false;
int melodyIdx = 0;
unsigned long noteStart = 0;
unsigned long noteDur = 0;
unsigned long nextMelodyAt = 0;

// distance
float distHist[4] = {MAX_CM, MAX_CM, MAX_CM, MAX_CM};  // newest at [3]
float distNow = MAX_CM;
float approachSpeed = 0;  // cm/s, positive = getting closer
unsigned long lastDistRead = 0;

// hiding
unsigned long closedAt = 0;  // 0 = not fully closed yet

// button
bool lastButtonState = HIGH;
unsigned long lastButtonChange = 0;

unsigned long lastPrint = 0;

// ---------------- Servo (non-blocking, slow) ----------------
void updateServo() {
  if (servoPos == servoTarget) return;
  int stepMs = (servoTarget < servoPos) ? CLOSE_STEP_MS : OPEN_STEP_MS;
  if (millis() - lastServoStep >= (unsigned long)stepMs) {
    lastServoStep = millis();
    servoPos += (servoTarget > servoPos) ? 1 : -1;
    flower.write(servoPos);
  }
}

// ---------------- Melody (non-blocking) ----------------
void playCurrentNote() {
  int divider = melody[melodyIdx + 1];
  noteDur = wholenote / abs(divider);
  if (divider < 0) noteDur = noteDur * 3 / 2;  // dotted
  tone(BUZZER_PIN, melody[melodyIdx] * octaveMultiplier);
  noteSounding = true;
  noteStart = millis();
}

void startMelody() {
  melodyIdx = 0;
  melodyPlaying = true;
  playCurrentNote();
}

void stopMelody() {
  melodyPlaying = false;
  noteSounding = false;
  noTone(BUZZER_PIN);
}

void updateMelody() {
  if (!melodyPlaying) return;
  unsigned long elapsed = millis() - noteStart;

  if (noteSounding && elapsed >= noteDur * articulation) {
    noTone(BUZZER_PIN);
    noteSounding = false;
  }
  if (elapsed >= noteDur) {
    melodyIdx += 2;
    if (melodyIdx >= notes * 2) {
      stopMelody();
      nextMelodyAt = LOOP_MELODY ? millis() + LOOP_PAUSE_MS : 0xFFFFFFFF;
      return;
    }
    playCurrentNote();
  }
}

// ---------------- Distance ----------------
float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return MAX_CM;  // no echo = nobody close
  float cm = duration * 0.0343 / 2.0;
  return (cm > MAX_CM) ? MAX_CM : cm;
}

void updateSensors() {
  if (millis() - lastDistRead >= DIST_INTERVAL_MS) {
    lastDistRead = millis();
    for (int i = 0; i < 3; i++) distHist[i] = distHist[i + 1];
    distHist[3] = readDistanceCm();
    distNow = distHist[3];
    // compare against reading 300 ms ago
    approachSpeed = (distHist[0] - distHist[3]) / 0.3;
  }
}

bool isBright() { return analogRead(LIGHT_PIN) > LIGHT_THRESHOLD; }  // flip to < if reversed

bool isScary() {
  // ignore dropouts (300 = no echo) so they can't fake a fast approach
  return distNow < SCARE_RANGE_CM && distHist[0] < MAX_CM && approachSpeed > FAST_APPROACH_CM_S;
}

// someone far enough away (backed off), and not moving around quickly
bool personBackedOff() {
  return distNow >= SAFE_DISTANCE_CM && fabs(approachSpeed) < CALM_SPEED_CM_S;
}

// ---------------- State changes ----------------
void goHide() {
  stopMelody();
  servoTarget = CLOSED_POS;
  closedAt = (servoPos == CLOSED_POS) ? millis() : 0;
  state = HIDING;
  Serial.println(">> Scared! Hiding.");
}

void goWake() {
  servoTarget = OPEN_POS;
  nextMelodyAt = 0;
  state = AWAKE;
  Serial.println(">> Coming out.");
}

void goSleep() {
  stopMelody();
  servoTarget = CLOSED_POS;
  state = SLEEPING;
  Serial.println(">> Button: closing for the end.");
}

// ---------------- Button ----------------
bool buttonPressed() {
  bool reading = digitalRead(BUTTON_PIN);
  bool pressed = false;
  if (reading != lastButtonState && millis() - lastButtonChange > 50) {
    lastButtonChange = millis();
    lastButtonState = reading;
    if (reading == LOW) pressed = true;
  }
  return pressed;
}

// ---------------- Arduino ----------------
void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  flower.attach(SERVO_PIN);
  flower.write(CLOSED_POS);
  servoPos = CLOSED_POS;
  delay(500);

  goWake();  // start open if nobody's around
}

void loop() {
  updateSensors();
  updateServo();
  updateMelody();

  // Button: toggles between "the end" and waking back up
  if (buttonPressed()) {
    if (state == SLEEPING) {
      goWake();
    } else {
      goSleep();
    }
  }

  switch (state) {
    case AWAKE:
      if (isScary()) {
        goHide();
        break;
      }
      // start melody once fully open
      if (!melodyPlaying && servoPos == OPEN_POS && millis() >= nextMelodyAt) {
        startMelody();
      }
      break;

    case HIDING:
      // start the 5 s timer once fully closed; restart it if scared again
      if (servoPos == CLOSED_POS && closedAt == 0) closedAt = millis();
      if (isScary() && servoPos == CLOSED_POS) closedAt = millis();

      if (closedAt != 0 && millis() - closedAt >= MIN_CLOSED_MS) {
        if (isBright() || personBackedOff()) goWake();
      }
      break;

    case SLEEPING:
      // closed and done; waiting for button
      break;
  }

  // debug output twice a second
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    Serial.print("state=");
    Serial.print(state == AWAKE ? "AWAKE" : state == HIDING ? "HIDING" : "SLEEPING");
    Serial.print(" dist=");
    Serial.print(distNow, 0);
    Serial.print("cm speed=");
    Serial.print(approachSpeed, 0);
    Serial.print("cm/s light=");
    Serial.println(analogRead(LIGHT_PIN));
  }
}