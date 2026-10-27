// Two-wheel wall-avoiding robot for the corner arena.
// Drives forward; when the front ultrasonic sensor sees a wall,
// backs up briefly, turns right in place, then resumes forward drive.
// Green LED on pin 13 is ON while driving forward, blinks while avoiding.
// Potentiometer on A0 controls the blink speed (avoiding blink rate).
//
// Wiring (diagram.json):
//   motor_left   en=5 (PWM) in1=4 in2=7
//   motor_right  en=6 (PWM) in1=8 in2=12
//   ultrasonic   trig=9 echo=10 (forward facing)
//   led          pin=13
//   potentiometer pin=A0 (blink speed)

const int L_EN = 5, L_IN1 = 4, L_IN2 = 7;
const int R_EN = 6, R_IN1 = 8, R_IN2 = 12;
const int TRIG = 9, ECHO = 10;
const int LED = 13;
const int POT = A0;

// Tunables
const int CRUISE = 200;        // forward speed 0..255
const int TURN_SPEED = 200;    // spin speed
const long WALL_CM = 35;       // start avoiding when closer than this
const long CLEAR_CM = 45;      // resume forward when clearer than this
const unsigned long BACKUP_MS = 350;
const unsigned long TURN_MS = 900;
const unsigned long PING_TIMEOUT_US = 30000UL;  // ~5 m max

// Blink speed range controlled by the potentiometer (A0).
// Pot fully CCW (0)   -> fast blink, 60 ms half-period.
// Pot fully CW (1023) -> slow blink, 500 ms half-period.
const unsigned long BLINK_FAST_MS = 60;
const unsigned long BLINK_SLOW_MS = 500;

enum State { DRIVE, BACKUP, TURN };
State state = DRIVE;
unsigned long stateStart = 0;
unsigned long lastBlink = 0;
bool ledOn = false;
unsigned long lastPotPrint = 0;

void motorWrite(int en, int in1, int in2, int speed) {
  // speed: -255..255
  if (speed >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(en, speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(en, -speed);
  }
}

void driveForward(int s) {
  motorWrite(L_EN, L_IN1, L_IN2, s);
  motorWrite(R_EN, R_IN1, R_IN2, s);
}

void driveBackward(int s) {
  motorWrite(L_EN, L_IN1, L_IN2, -s);
  motorWrite(R_EN, R_IN1, R_IN2, -s);
}

void spinRight(int s) {
  // left wheel forward, right wheel backward -> turn in place
  motorWrite(L_EN, L_IN1, L_IN2, s);
  motorWrite(R_EN, R_IN1, R_IN2, -s);
}

void stopMotors() {
  analogWrite(L_EN, 0);
  analogWrite(R_EN, 0);
}

long readDistanceCm() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  unsigned long dur = pulseIn(ECHO, HIGH, PING_TIMEOUT_US);
  if (dur == 0) return 999;  // nothing in range -> treat as clear
  return (long)(dur / 58UL);
}

// Read the potentiometer and convert to a blink half-period in ms.
// 0..1023 maps to BLINK_FAST_MS..BLINK_SLOW_MS.
unsigned long blinkIntervalMs() {
  int potVal = analogRead(POT);
  long ms = map(potVal, 0, 1023, (long)BLINK_FAST_MS, (long)BLINK_SLOW_MS);
  return (unsigned long)constrain(ms, (long)BLINK_FAST_MS, (long)BLINK_SLOW_MS);
}

void setup() {
  Serial.begin(115200);
  pinMode(L_EN, OUTPUT); pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(R_EN, OUTPUT); pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(TRIG, LOW);
  stopMotors();
  state = DRIVE;
  Serial.println("ready");
}

void loop() {
  long dist = readDistanceCm();
  unsigned long now = millis();
  unsigned long blinkMs = blinkIntervalMs();

  // Report pot + blink rate ~once per second so the knob effect is visible
  // in serial logs and verifiable in simulation.
  if (now - lastPotPrint >= 1000) {
    lastPotPrint = now;
    Serial.print("pot ");
    Serial.print(analogRead(POT));
    Serial.print(" blink ");
    Serial.print(blinkMs);
    Serial.println("ms");
  }

  switch (state) {
    case DRIVE:
      if (dist < WALL_CM) {
        state = BACKUP;
        stateStart = now;
        lastBlink = now;
        ledOn = false;
        driveBackward(170);
        Serial.print("wall at ");
        Serial.print(dist);
        Serial.println("cm: backup");
      } else {
        // Gentle slowdown as we approach the threshold so we don't
        // overshoot into the wall.
        int s = CRUISE;
        if (dist < CLEAR_CM + 20) {
          s = map((int)dist, (int)WALL_CM, (int)(CLEAR_CM + 20), 130, CRUISE);
          s = constrain(s, 130, CRUISE);
        }
        driveForward(s);
        digitalWrite(LED, HIGH);
        ledOn = true;
      }
      break;

    case BACKUP:
      // Blink at the potentiometer-controlled rate (non-blocking).
      if (now - lastBlink >= blinkMs) {
        lastBlink = now;
        ledOn = !ledOn;
        digitalWrite(LED, ledOn ? HIGH : LOW);
      }
      if (now - stateStart >= BACKUP_MS) {
        state = TURN;
        stateStart = now;
        spinRight(TURN_SPEED);
        Serial.println("turn");
      }
      break;

    case TURN:
      // Same potentiometer-controlled blink while turning.
      if (now - lastBlink >= blinkMs) {
        lastBlink = now;
        ledOn = !ledOn;
        digitalWrite(LED, ledOn ? HIGH : LOW);
      }
      if (now - stateStart >= TURN_MS) {
        state = DRIVE;
        driveForward(CRUISE);
        Serial.println("forward");
      }
      break;
  }

  delay(20);
}
