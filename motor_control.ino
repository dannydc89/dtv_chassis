#define NANO

#ifdef NANO
  int DIR_PIN_OUT  = 10;
  int HALL_PIN     = 2;
  int PWM_PIN_OUT  = 3;
  int TEST_PIN_OUT = 12; 
  const int PIN_BRAKE = 4;
#endif

// ── Interrupt & Timing variables ─────────────────────────────────────────────
volatile unsigned long tic = 0, tac = 0;
volatile unsigned long feedback_raw_us = 0; 
volatile unsigned long last_pulse_time = 0; 

// ── Mechanical Constants ─────────────────────────────────────────────────────
const int   PULSES_PER_REV = 45;            
const float WHEEL_CIRCUMFERENCE_M = 0.911;  

// ── Speed variables ──────────────────────────────────────────────────────────
float speed_pps = 0.0; 
float speed_rpm = 0.0; 
float speed_ms  = 0.0; 

// ── PID & Soft Start variables ───────────────────────────────────────────────
float target_user = 60.0; 
float target_unit = 1.0;  // 0=pps, 1=RPM, 2=m/s
float target_pps  = 0.0;
float target_ramped_pps = 0.0; // Target used by PID (interpolated)

float acceleration = 10.0; // Rate of change in pps/sec. Increase for faster response.

float kp = 1.0, ki = 0.5, kd = 0.02;
float P = 0, I = 0, D = 0;
float error = 0, prevErr = 0, errSum = 0;
float maxSum = 100.0; 

float pwm_min = 15.0; 
float pwm_max = 255.0;
float test_disable = 0; 

unsigned long lastLoopTime = 0;
float dt = 0;

struct CommandRef { float* varPtr; const char* label; };
CommandRef lut[128] = { {nullptr, nullptr} };

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  initSerialCommands();

  pinMode(DIR_PIN_OUT,  OUTPUT);
  pinMode(PWM_PIN_OUT,  OUTPUT);
  pinMode(TEST_PIN_OUT, OUTPUT);
  pinMode(HALL_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(HALL_PIN), intrupt, RISING);

  digitalWrite(DIR_PIN_OUT, 1); 
  analogWrite(PWM_PIN_OUT, 0);

  Serial.println("--- BLDC Controller with Soft Start ---");
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  handleSerialCommands();

  unsigned long currentMillis = millis();
  dt = (currentMillis - lastLoopTime) / 1000.0;
  if (dt <= 0) dt = 0.001; 
  lastLoopTime = currentMillis;

  // 1. Thread-safe copy of interrupt data
  noInterrupts();
  unsigned long fb_us = feedback_raw_us;
  unsigned long last_p = last_pulse_time;
  interrupts();

  // 2. Calculate actual speed
  if (millis() - last_p > 500 || fb_us == 0) {
    speed_pps = 0.0;
  } else {
    speed_pps = 1000000.0 / (float)fb_us; 
  }

  speed_rpm = (speed_pps * 60.0) / PULSES_PER_REV;
  speed_ms  = (speed_pps / PULSES_PER_REV) * WHEEL_CIRCUMFERENCE_M;

  // 3. SOFT START LOGIC (Ramp Generator)
  target_pps = convertTargetToPPS(target_user, (int)target_unit);

  if (target_ramped_pps < target_pps) {
    target_ramped_pps += acceleration * dt;
    if (target_ramped_pps > target_pps) target_ramped_pps = target_pps;
  } else if (target_ramped_pps > target_pps) {
    target_ramped_pps -= acceleration * dt;
    if (target_ramped_pps < target_pps) target_ramped_pps = target_pps;
  }

  // 4. PID Calculation (uses target_ramped_pps instead of target_pps)
  error = target_ramped_pps - speed_pps;
  P = kp * error;
  errSum = constrain(errSum + (error * dt), -maxSum, maxSum);
  I = ki * errSum;
  D = kd * (error - prevErr) / dt;
  prevErr = error;
  
  float pid_output = P + I + D;

  // 5. Motor output
  if (test_disable == 0 && target_user > 0) {
    int pwm_val = (int)constrain(pid_output, pwm_min, pwm_max);
    analogWrite(PWM_PIN_OUT, pwm_val);
  } else {
    analogWrite(PWM_PIN_OUT, 0);
    errSum = 0; 
    target_ramped_pps = 0; // Reset ramp when motor is off
  }

  Plotter(pid_output);
}

// ─────────────────────────────────────────────────────────────────────────────
void intrupt() {
  tic = micros();
  if (tac != 0) {
    feedback_raw_us = tic - tac;
  }
  tac = tic;
  last_pulse_time = millis();
  digitalWrite(TEST_PIN_OUT, !digitalRead(TEST_PIN_OUT));
}

float convertTargetToPPS(float user_val, int unit) {
  switch(unit) {
    case 0: return user_val; 
    case 1: return (user_val / 60.0) * PULSES_PER_REV; 
    case 2: return (user_val / WHEEL_CIRCUMFERENCE_M) * PULSES_PER_REV; 
    default: return user_val;
  }
}

void Plotter(float p_out) {
  static unsigned long lastPlot = 0;
  if (millis() - lastPlot < 50) return; 
  lastPlot = millis();

  Serial.print("Target_Ramped:"); Serial.print(target_ramped_pps, 1); Serial.print(" ");
  Serial.print("Real_PPS:");      Serial.print(speed_pps, 1); Serial.print(" ");
  Serial.print("RPM:");           Serial.print(speed_rpm, 1); Serial.print(" ");
  Serial.print("PWM:");           Serial.println(constrain(p_out, pwm_min, pwm_max));
}

void initSerialCommands() {
  lut['V'].varPtr = &target_user;  lut['V'].label = "Target";
  lut['U'].varPtr = &target_unit;  lut['U'].label = "Unit";
  lut['p'].varPtr = &kp;           lut['p'].label = "Kp";
  lut['i'].varPtr = &ki;           lut['i'].label = "Ki";
  lut['d'].varPtr = &kd;           lut['d'].label = "Kd";
  lut['a'].varPtr = &acceleration; lut['a'].label = "Accel"; // New command for acceleration
  lut['m'].varPtr = &pwm_min;      lut['m'].label = "PWM_Min";
  lut['D'].varPtr = &test_disable; lut['D'].label = "Disable";
}

void handleSerialCommands() {
  if (Serial.available() > 0) {
    int cmdChar = Serial.read();
    if (cmdChar == 'R') {
      errSum = 0; prevErr = 0; target_ramped_pps = 0;
      Serial.println("Reset Done.");
      return;
    }
    if (cmdChar < 128 && lut[cmdChar].varPtr != nullptr) {
      float val = Serial.parseFloat();
      *(lut[cmdChar].varPtr) = val;
      Serial.print(lut[cmdChar].label); Serial.print("="); Serial.println(val);
    }
  }
}