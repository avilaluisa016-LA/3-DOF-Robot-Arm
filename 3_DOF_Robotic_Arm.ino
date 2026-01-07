// ---- Libraries ----
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

// ---- Servo Objects ----
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);  // This is the default address

const byte rServo = 0, T1Servo = 1, T2Servo = 2;  // Servos and channel
const float PWM_Freq = 50.0;

// ---- JoyStick ----
const byte xPin = A0;
const byte yPin = A1;
const byte bPin = 12;

uint16_t yVal, zVal;

// ---- Arm Lengths ----
const float L1 = 3.5;
const float L2 = 2;

// ---- Servo limits ----
const float R_ANGLE_MIN = -90.0, R_ANGLE_MAX = 90.0;
const float T1_ANGLE_MIN = -45.0, T1_ANGLE_MAX = 45.0;
const float T2_ANGLE_MIN = -90.0, T2_ANGLE_MAX = 90.0;

const float R_PULSE_MIN = 500, R_PULSE_MAX = 2500;
const float T1_PULSE_MIN = 500, T1_PULSE_MAX = 2500;
const float T2_PULSE_MIN = 500, T2_PULSE_MAX = 2500;

uint16_t usToCounts(float us){  // microseconds -> count
  return (uint16_t)((us * 4096.0) / 20000.0);
}

// ---- RServo ----
float  angleToPulse_R(float angleDeg){  // angle -> pulse -> PWM
  angleDeg = constrain(angleDeg, R_ANGLE_MIN, R_ANGLE_MAX);
  return R_PULSE_MIN +(angleDeg - R_ANGLE_MIN) * (R_PULSE_MAX - R_PULSE_MIN) / (R_ANGLE_MAX - R_ANGLE_MIN);
}

void setRAngle(float angleDeg) {
  float pulseUs = angleToPulse_R(angleDeg);
  uint16_t counts = usToCounts(pulseUs);
  pwm.setPWM(rServo, 0, counts);
}

// ---- T1Servo ----
float  angleToPulse_T1(float angleDeg){  // angle -> pulse -> PWM
  angleDeg = constrain(angleDeg, T1_ANGLE_MIN, T1_ANGLE_MAX);
  return T1_PULSE_MIN + (angleDeg - T1_ANGLE_MIN) * (T1_PULSE_MAX - T1_PULSE_MIN) / (T1_ANGLE_MAX - T1_ANGLE_MIN);
}

void setT1Angle(float angleDeg) {
  float pulseUs = angleToPulse_T1(angleDeg);
  uint16_t counts = usToCounts(pulseUs);
  pwm.setPWM(T1Servo, 0, counts);
}

// ---- T2Servo ----
float  angleToPulse_T2(float angleDeg){  // angle -> pulse -> PWM
  angleDeg = constrain(angleDeg, T2_ANGLE_MIN, T2_ANGLE_MAX);
  return T2_PULSE_MIN + (angleDeg - T2_ANGLE_MIN) * (T2_PULSE_MAX - T2_PULSE_MIN) / (T2_ANGLE_MAX - T2_ANGLE_MIN);
}

void setT2Angle(float angleDeg) {
  float pulseUs = angleToPulse_T2(angleDeg);
  uint16_t counts = usToCounts(pulseUs);
  pwm.setPWM(T2Servo, 0, counts);
}

// ---- Inverse Kinematics ----
bool computeIK(float y, float z, float &T1_deg, float &T2_deg) {
  float h = sqrt(y*y + z*z);
  float reach = L1 + L2;

  if (h > reach || h < fabs(L1 - L2)) return false;

  // --- Elbow angle (law of cosines) ---
  float cosT2 = (h*h - L1*L1 - L2*L2) / (2 * L1 * L2);
  cosT2 = constrain(cosT2, -1.0, 1.0);
  float T2_rad = acos(cosT2);

  // --- Shoulder angle ---
  float gamma = atan2(z, y);
  float delta = atan2(L2 * sin(T2_rad), L1 + L2 * cos(T2_rad));
  float T1_rad = gamma - delta;

  T1_deg = T1_rad * 180.0 / PI;
  T2_deg = T2_rad * 180.0 / PI;

  return true;
}

// ---- Smooth movement ----
float R_curr  = 90.0;
float T1_curr = 45.0;
float T2_curr = 90.0;

// Smoothing factor (0.05–0.3 recommended)
const float SMOOTHING = 0.12;

float smoothAngle(float current, float target) {
  return current + (target - current) * SMOOTHING;
}

bool initialized = false;  // Used to set servosto a home position

void setup() {
  // ---- PCA9685 ----
  pwm.begin();
  pwm.setPWMFreq(PWM_Freq);

  // ---- Joystick ----
  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);
  pinMode(bPin, INPUT_PULLUP);

  // ---- Move to home position ----
  setRAngle(R_curr);
  setT1Angle(T1_curr);
  setT2Angle(T2_curr);

  delay(500);  // allow servos to settle


}  // setup()

void loop() {
  int xVal = analogRead(xPin);
  int yVal = analogRead(yPin);

  // ---- Deadzone ----
  if (abs(xVal - 512) < 20) xVal = 512;
  if (abs(yVal - 512) < 20) yVal = 512;

  // ---- Target angles ----
  float R_target = map(xVal, 0, 1023, R_ANGLE_MIN, R_ANGLE_MAX);

  float yTarget = L1;
  float zTarget = map(yVal, 0, 1023, 0, L1 + L2);

  float T1_target = T1_curr;
  float T2_target = T2_curr;

  if (!initialized) {
    setRAngle(R_curr);
    setT1Angle(T1_curr);
    setT2Angle(T2_curr);
    initialized = true;
    return;
  }

  if (computeIK(yTarget, zTarget, T1_target, T2_target)) {
    // Smooth transitions
    R_curr  = smoothAngle(R_curr,  R_target);
    T1_curr = smoothAngle(T1_curr, T1_target);
    T2_curr = smoothAngle(T2_curr, T2_target);

    setRAngle(R_curr);
    setT1Angle(T1_curr);
    setT2Angle(T2_curr);
  }
  delay(20);
}  // loop()
