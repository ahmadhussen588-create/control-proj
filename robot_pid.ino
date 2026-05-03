// ===================== PIN DEFINITIONS =====================
#define TRIG_F  2
#define ECHO_F  3
#define TRIG_L  4
#define ECHO_L  7
#define TRIG_R  8
#define ECHO_R  12

#define ENA  6
#define IN1  11
#define IN2  13
#define ENB  5
#define IN3  9
#define IN4  10

// ===================== CALIBRATED VALUES =====================
const int BASE_SPEED      = 150;   // base PWM for forward driving (used by PID)
const int LEFT_TRIM       = 13;    // LEFT_SPEED - RIGHT_SPEED bias from your calibration (163-130 ~ +13 boost on left)
const int MIN_PWM         = 70;    // motors won't move below this
const int MAX_PWM         = 230;   // cap so we never saturate fully and lose control
const int TURN_PWM        = 255;
const int TURN_RIGHT_MS   = 170;
const int TURN_LEFT_MS    = 170;
const int BACKUP_MS       = 500;

// ===================== PID TUNING (FILL IN YOUR VALUES) =====================
// Error = (L - R). Positive error => robot is closer to RIGHT wall => steer LEFT.
float Kp = 0.0;   // <-- ADD YOUR Kp HERE  (try starting around 4.0 - 8.0)
float Ki = 0.0;   // <-- ADD YOUR Ki HERE  (try starting around 0.0 - 0.05)
float Kd = 0.0;   // <-- ADD YOUR Kd HERE  (try starting around 1.0 - 3.0)

// PID state
float pidIntegral   = 0.0;
float pidPrevError  = 0.0;
unsigned long pidPrevMs = 0;
const float I_CLAMP = 200.0;       // anti-windup clamp

// ===================== SAFETY THRESHOLDS =====================
const float FRONT_STOP    = 15.0;  // hard stop distance (raised from 10 for safety margin)
const float FRONT_SLOW    = 35.0;  // start slowing earlier
const float SIDE_DANGER   = 8.0;   // emergency veer-away if a side wall is this close
const float SIDE_TARGET   = 15.0;  // desired distance from each side wall (centering target)
const float MAX_RANGE     = 200.0;
const int   LOOP_DELAY_MS = 20;

float prevR = 200.0;

// ===================== SENSOR =====================
float readUltrasonic(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long d = pulseIn(echo, HIGH, 20000UL);
  if (d == 0) return MAX_RANGE;
  float cm = d * 0.0343f / 2.0f;
  return cm > MAX_RANGE ? MAX_RANGE : cm;
}

// Median-of-3 read for noise rejection (reduces spurious readings that cause crashes)
float readUltrasonicFiltered(int trig, int echo) {
  float a = readUltrasonic(trig, echo);
  delayMicroseconds(500);
  float b = readUltrasonic(trig, echo);
  delayMicroseconds(500);
  float c = readUltrasonic(trig, echo);
  // median
  if ((a >= b && a <= c) || (a <= b && a >= c)) return a;
  if ((b >= a && b <= c) || (b <= a && b >= c)) return b;
  return c;
}

void readAll(float &f, float &l, float &r) {
  f = readUltrasonicFiltered(TRIG_F, ECHO_F); delay(8);
  l = readUltrasonicFiltered(TRIG_L, ECHO_L); delay(8);
  r = readUltrasonicFiltered(TRIG_R, ECHO_R); delay(8);
}

// ===================== MOTOR CONTROL =====================
void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); analogWrite(ENB, 0);
}

int clampPWM(int v) {
  if (v < MIN_PWM) return MIN_PWM;
  if (v > MAX_PWM) return MAX_PWM;
  return v;
}

// Drive forward with independent left/right PWM (PID drives this)
void driveForwardPWM(int leftPWM, int rightPWM) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, clampPWM(leftPWM));
  analogWrite(ENB, clampPWM(rightPWM));
}

void backup() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, BASE_SPEED + LEFT_TRIM);
  analogWrite(ENB, BASE_SPEED);
  delay(BACKUP_MS);
  stopMotors();
  delay(300);
}

// ===================== TURNS =====================
void turnRight() {
  stopMotors(); delay(200);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  analogWrite(ENA, TURN_PWM);
  analogWrite(ENB, TURN_PWM);
  delay(TURN_RIGHT_MS);
  stopMotors(); delay(200);
}

void turnLeft() {
  stopMotors(); delay(200);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, TURN_PWM);
  analogWrite(ENB, TURN_PWM);
  delay(TURN_LEFT_MS);
  stopMotors(); delay(200);
}

void uTurn() {
  backup();
  turnLeft();
  delay(100);
  turnLeft();
}

void resetPID() {
  pidIntegral  = 0.0;
  pidPrevError = 0.0;
  pidPrevMs    = millis();
}

// ===================== PID CENTERING =====================
// Returns the steering correction in PWM units.
// Positive correction -> turn left (slow left, speed right) i.e. push away from right wall.
float computePIDCorrection(float l, float r) {
  // If a side is out of range (open corridor), substitute SIDE_TARGET so PID
  // doesn't go crazy chasing a phantom far wall.
  float lEff = (l >= MAX_RANGE) ? SIDE_TARGET : l;
  float rEff = (r >= MAX_RANGE) ? SIDE_TARGET : r;

  float error = lEff - rEff;   // >0 => closer to right wall

  unsigned long now = millis();
  float dt = (now - pidPrevMs) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f;
  pidPrevMs = now;

  pidIntegral += error * dt;
  if (pidIntegral >  I_CLAMP) pidIntegral =  I_CLAMP;
  if (pidIntegral < -I_CLAMP) pidIntegral = -I_CLAMP;

  float derivative = (error - pidPrevError) / dt;
  pidPrevError = error;

  return Kp * error + Ki * pidIntegral + Kd * derivative;
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(9600);
  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors();
  delay(3000);
  prevR = 200.0;
  resetPID();
}

// ===================== MAIN LOOP =====================
void loop() {
  float f, l, r;
  readAll(f, l, r);

  Serial.print("F:"); Serial.print(f);
  Serial.print(" L:"); Serial.print(l);
  Serial.print(" R:"); Serial.println(r);

  // ===== PRIORITY 1: FRONT WALL (hard stop, no collision) =====
  if (f < FRONT_STOP) {
    stopMotors();
    delay(300);
    if (f < 10.0) backup();
    readAll(f, l, r);
    Serial.print("DECIDING L:"); Serial.print(l);
    Serial.print(" R:"); Serial.println(r);
    if (r > l) {
      Serial.println("TURN RIGHT");
      turnRight();
    } else if (l > r) {
      Serial.println("TURN LEFT");
      turnLeft();
    } else {
      Serial.println("UTURN");
      uTurn();
    }
    prevR = 200.0;
    resetPID();
    return;
  }

  // ===== PRIORITY 2: SIDE EMERGENCY (about to scrape a wall) =====
  if (l < SIDE_DANGER) {
    Serial.println("EMERGENCY VEER RIGHT");
    // pivot away from left wall briefly
    driveForwardPWM(BASE_SPEED + 40, MIN_PWM);
    delay(80);
    resetPID();
    return;
  }
  if (r < SIDE_DANGER) {
    Serial.println("EMERGENCY VEER LEFT");
    driveForwardPWM(MIN_PWM, BASE_SPEED + 40);
    delay(80);
    resetPID();
    return;
  }

  // ===== PRIORITY 3: RIGHT CORRIDOR OPENING =====
  if (prevR < 80.0 && r > 80.0 && f > FRONT_STOP) {
    Serial.println("RIGHT OPENING DETECTED");
    stopMotors();
    delay(100);
    driveForwardPWM(BASE_SPEED + LEFT_TRIM, BASE_SPEED);
    delay(150);
    stopMotors();
    delay(100);
    turnRight();
    prevR = 200.0;
    resetPID();
    return;
  }
  prevR = r;

  // ===== PRIORITY 4: PID WALL-CENTERING =====
  // Speed scales down as front wall approaches (smooth braking, no collisions)
  float speedScale = 1.0;
  if (f < FRONT_SLOW) {
    speedScale = (f - FRONT_STOP) / (FRONT_SLOW - FRONT_STOP);
    if (speedScale < 0.35) speedScale = 0.35;
    if (speedScale > 1.0)  speedScale = 1.0;
  }

  int base = (int)(BASE_SPEED * speedScale);
  float correction = computePIDCorrection(l, r);

  // Positive correction -> error positive (closer to right) -> steer left:
  //   slow LEFT motor, speed up RIGHT motor.
  int leftPWM  = base + LEFT_TRIM - (int)correction;
  int rightPWM = base              + (int)correction;

  driveForwardPWM(leftPWM, rightPWM);

  delay(LOOP_DELAY_MS);
}
