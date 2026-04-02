// ═══════════════════════════════════════════════════════════════════════════
// DIFFERENTIAL DRIVE — Closed-loop control + PID Auto-Tuner (Ziegler-Nichols)
// ═══════════════════════════════════════════════════════════════════════════
//
// DRIVE COMMANDS:
//   V<val>   — target speed             U<0/1/2> — unit (pps / RPM / m/s)
//   O<val>   — turn offset              r<val>   — ramp rate (pps/loop)
//   S<0/1>   — sync enable              R        — full reset
//   A<0/1>   — advanced features off/on M<0/1>  — direct PWM mode off/on
//   q<val>   — left direct duty (%)     Q<val>  — right direct duty (%)
//   e<0/1>   — disable both motors
//   EL<0/1>  — enable/disable LEFT motor only
//   ER<0/1>  — enable/disable RIGHT motor only
//
// PID COMMANDS:
//   p/i/d    — Left motor Kp/Ki/Kd      P/I/D    — Right motor Kp/Ki/Kd
//   x/y      — Sync Kp/Ki
//   f<val>   — Left feedforward PWM offset  (compensates hardware asymmetry)
//   F<val>   — Right feedforward PWM offset
//
// AUTO-TUNER COMMANDS:
//   TL       — start tuning LEFT motor  (right motor stopped during tuning)
//   TR       — start tuning RIGHT motor (left motor stopped during tuning)
//   TQ       — abort tuning
//
// NOTE: After tuning, coefficients are applied automatically.
//       Send R to return to normal drive mode.
// ═══════════════════════════════════════════════════════════════════════════

#define NANO
#define MOTOR_LEFT_ENABLED
#define MOTOR_RIGHT_ENABLED

const char FIRMWARE_VERSION[] = "motor-control 2026-04-02 pid-v10";

// ── Pin definitions ───────────────────────────────────────────────────────────
#ifdef NANO
  const int DIR_PIN_LEFT   = 10;
  const int PWM_PIN_LEFT   = 5;
  const int HALL_PIN_LEFT  = 2;   // Must be interrupt pin
  const int DIR_PIN_RIGHT  = 9;
  const int PWM_PIN_RIGHT  = 6;   // Must be interrupt pin
  const int HALL_PIN_RIGHT = 3;
  const int TEST_PIN       = 12;  // Toggles on each left Hall pulse (debug)
  const int PIN_BRAKE      = 4;   // Brake pin (unused)
#endif

// ── Motor constants ───────────────────────────────────────────────────────────
const int   PULSES_PER_REV      = 45;    // Hall pulses per full wheel revolution
const float WHEEL_CIRCUMFERENCE = 0.52; // Wheel circumference in meters (6.5" hoverboard)

// ── Timeouts ──────────────────────────────────────────────────────────────────
const unsigned long MOTOR_TIMEOUT_MS = 500; // No Hall pulse within this period -> assume stopped

// ── Safety limits ─────────────────────────────────────────────────────────────
const float MAX_SAFE_PPS     = 300.0;   // Maximum allowed speed (pps)
const float MAX_SAFE_ACCEL   = 200.0;   // Maximum allowed acceleration (pps/s)
const float MAX_PID_OUTPUT   = 1000.0;  // PID output above this = runaway
const int   PWM_RAMP_LIMIT   = 30;      // Max PWM change per loop cycle
const int   PWM_START_MIN    = 12;      // Minimum PWM used only to break static friction at startup
const int   PWM_START_BOOST  = 26;      // Short startup kick to get the wheel moving
const unsigned long PWM_START_BOOST_MS = 180; // Limit startup kick to a brief pulse
const unsigned long STARTUP_SAFETY_GRACE_MS = 350; // Ignore startup spikes briefly after launch
const float PID_ERRSUM_LIMIT_ADV = 120.0f;    // Larger I-term headroom for reaching cruise speed
const float PID_ERRSUM_LIMIT_SIMPLE = 160.0f; // Simple mode needs a little extra headroom too
const float PID_STARTUP_I_SCALE = 0.35f;      // Integrate more gently while wheel is still breaking free
const float SPEED_FILTER_ALPHA = 0.35f;       // Low-pass filter for Hall-derived speed estimate
const float SYNC_CLAMP_RATIO = 0.8f;    // Sync can adjust target by at most 80%
const float MAX_HALL_PPS     = 1000.0;  // Above this = noise/floating pin, filtered out
const float STARTUP_SPEED_PPS = 1.5f;   // Below this speed the controller may need startup assist
const float LOAD_FF_TARGET_MIN_PPS = 18.0f;   // Learn extra drive only above this speed target
const float LOAD_FF_MAX = 8.0f;               // Max extra PWM shared by both motors under load
const float LOAD_FF_UP_RATE = 0.35f;          // Learn additional PWM when average speed stays below target
const float LOAD_FF_DOWN_RATE = 0.90f;        // Forget extra PWM when no longer needed
const float LOAD_FF_IDLE_DECAY = 3.20f;       // Bleed learned assist away at low target speeds
const float LOAD_FF_SYNC_GATE = 0.20f;        // Freeze learning if sync correction grows too large

// ── Auto-tuner settings ───────────────────────────────────────────────────────
const int   TUNE_PWM_INIT   = 40;    // Initial open-loop PWM
const int   TUNE_PWM_MAX    = 180;   // Max PWM allowed during tuning
const float TUNE_TARGET_PPS = 45.0;  // Speed target for tuning
const float TUNE_KP_START   = 0.1;   // Initial Kp for Ku search
const float TUNE_KP_STEP    = 0.1;   // Kp increment per step
const float TUNE_KP_MAX     = 20.0;  // Max Kp before aborting
const int   TUNE_SETTLE_MS  = 2000;  // Wait time after each Kp increase (ms)
const int   TUNE_OBSERVE_MS = 9000;  // Oscillation observation window (ms)
const int   TUNE_PHASE1_MS  = 15000; // Phase 1 timeout — open-loop stabilization (ms)
const float TUNE_OSC_THRESH = 3.0;   // Minimum oscillation amplitude to confirm (pps)
const int   TUNE_MIN_CYCLES = 3;     // Minimum oscillation cycles required

// ── Motor structure ───────────────────────────────────────────────────────────
struct Motor {
  // Pins
  int pin_pwm, pin_dir, pin_hall;

  // Interrupt data (volatile — written in ISR, read in loop)
  volatile unsigned long feedback_raw;     // us between last 2 Hall pulses
  volatile unsigned long last_pulse_time;  // timestamp of last Hall pulse (us)
  volatile float         counter;          // total pulse count (used for sync)

  // Speed (computed in loop from feedback_raw)
  float speed_pps, speed_rpm, speed_ms;

  // PID coefficients and state
  float kp, ki, kd;
  float errSum, prevErr, pid_out;

  // Control
  float target;      // target speed in pps
  bool  dir;         // commanded logical direction: true = forward, false = reverse
  bool  invert_dir;  // invert physical direction output for this motor
  float duty_cycle;  // current duty cycle (0-100%)

  // Safety tracking
  float         prev_speed_pps;    // previous speed for acceleration check
  unsigned long accel_check_time;  // timestamp of last acceleration check
  int           prev_pwm;          // previous PWM value for ramp limiting

  // Feedforward offset — fixed PWM bias added on top of PID output
  // Use to compensate hardware asymmetry between motors
  float ff_offset;
  int           startup_pwm_min;
  int           startup_pwm_boost;
  bool          startup_boost_active;
  unsigned long startup_boost_until;

  // Per-motor runtime enable flag
  bool enabled;
};

Motor motorL, motorR;

// ── Sync PID ──────────────────────────────────────────────────────────────────
float sync_kp = 0.06, sync_ki = 0.0, sync_kd = 0.0;
float sync_errSum = 0, sync_prevErr = 0, sync_out = 0, sync_enable = 1.0;

// ── User target ───────────────────────────────────────────────────────────────
float target_unit   = 1;    // 0=pps  1=RPM  2=m/s
float target_user   = 0;    // target as entered by user
float target_pps    = 0;    // target converted to pps (internal unit)
float target_ramped = 0;    // ramped target — increases gradually to avoid sudden start
float ramp_rate     = 0.8;  // pps per loop (0 = disabled)
float turn_offset   = 0.0;  // pps offset: positive = turn right, negative = turn left
float load_ff_boost = 0.0;  // adaptive shared PWM boost for heavier load

// ── Time tracking ─────────────────────────────────────────────────────────────
float now_f, prvTime, dt;
const unsigned long SERIAL_REPORT_INTERVAL_MS = 200;
unsigned long       last_serial_report_ms     = 0;
const unsigned long SERIAL_COMMAND_IDLE_MS    = 20;
const int           SERIAL_COMMAND_BUFFER_LEN = 96;
char                serial_command_buffer[SERIAL_COMMAND_BUFFER_LEN];
int                 serial_command_length     = 0;
unsigned long       serial_last_byte_ms       = 0;

// ── Diagnostics and debug modes ──────────────────────────────────────────────
float        test_disable             = 0;
unsigned int test_pin_value           = 0;
bool         emergency_active         = false;
float        advanced_features_enable = 1.0;  // 1 = original advanced logic enabled
float        direct_pwm_mode          = 0.0;  // 1 = bypass target/PID and use direct duty commands
float        direct_duty_left         = 0.0;  // -100..100 %, negative = reverse
float        direct_duty_right        = 0.0;  // -100..100 %, negative = reverse
float        serial_status_enable     = 0.0;  // 1 = periodic [STATUS] lines enabled

// ── Serial command lookup table ───────────────────────────────────────────────
struct CommandRef { float* varPtr; const char* label; };
CommandRef lut[128] = { {nullptr, nullptr} };

// ── Auto-tuner state ──────────────────────────────────────────────────────────
enum TuneState { TUNE_IDLE, TUNE_PHASE1, TUNE_PHASE2, TUNE_PHASE3, TUNE_DONE };
TuneState   tune_state      = TUNE_IDLE;
Motor*      tune_motor      = nullptr;  // pointer to motor being tuned
Motor*      tune_other      = nullptr;  // pointer to the other motor (stopped)
const char* tune_motor_name = nullptr;

float         tune_kp_current  = 0;
unsigned long tune_phase_start = 0;
float         tune_osc_min     = 1e6, tune_osc_max = -1e6;
float         tune_osc_last    = 0;
int           tune_osc_cross   = 0;
unsigned long tune_last_cross  = 0;
float         tune_period_sum  = 0;
int           tune_period_cnt  = 0;
float         tune_result_ku   = 0, tune_result_tu = 0;

// ── Function prototypes ───────────────────────────────────────────────────────
float convertToPPS(float v, int u);
float runPID(Motor& m, float target, float dt);
float runSyncPID(float dt);
void  updateSpeed(Motor& m);
void  applyPWM(Motor& m, float pid_out);
void  applyDirectDuty(Motor& m, float duty_percent);
void  stopMotor(Motor& m);
void  resetMotor(Motor& m);
void  updateLoadFeedforward(bool advanced_on, bool direct_on, float dt);
bool  safetyCheck(Motor& m, float dt);
void  emergencyStop(const __FlashStringHelper* reason);
void  Plotter();
void  printStatusLine();
void  initSerialCommands();
void  handleSerialCommands();
void  handleSerialCommandStream();
void  processSerialCommandBuffer();
bool  executeSerialCommand(char*& cursor);
bool  readSerialFloatArg(char*& cursor, float& value);
bool  readSerialIntArg(char*& cursor, int& value);
void  skipSerialSeparators(char*& cursor);
void  startTuning(Motor& m, Motor& other, const char* name);
void  runTuner();
void  abortTuning();
void  tunerComputeAndPrint();

// ── ISR — Left motor Hall sensor ──────────────────────────────────────────────
#ifdef MOTOR_LEFT_ENABLED
void isrLeft() {
  unsigned long t        = micros();
  motorL.feedback_raw    = t - motorL.last_pulse_time;
  motorL.last_pulse_time = t;
  motorL.counter++;
}
#endif

// ── ISR — Right motor Hall sensor ─────────────────────────────────────────────
#ifdef MOTOR_RIGHT_ENABLED
void isrRight() {
  unsigned long t        = micros();
  motorR.feedback_raw    = t - motorR.last_pulse_time;
  motorR.last_pulse_time = t;
  motorR.counter++;
}
#endif

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // ── Left motor defaults ──
  motorL.pin_pwm          = PWM_PIN_LEFT;
  motorL.pin_dir          = DIR_PIN_LEFT;
  motorL.pin_hall         = HALL_PIN_LEFT;
  motorL.kp               = 0.7;
  motorL.ki               = 0.10;
  motorL.kd               = 0.02;
  motorL.dir              = true;
  motorL.invert_dir       = false;
  motorL.feedback_raw     = 0;
  motorL.last_pulse_time  = 0;
  motorL.counter          = 0;
  motorL.errSum           = 0;
  motorL.prevErr          = 0;
  motorL.prev_speed_pps   = 0;
  motorL.accel_check_time = 0;
  motorL.prev_pwm         = 0;
  motorL.ff_offset        = 8;
  motorL.startup_pwm_min  = 12;
  motorL.startup_pwm_boost = 24;
  motorL.startup_boost_active = false;
  motorL.startup_boost_until  = 0;
  motorL.enabled          = true;

  // ── Right motor defaults ──
  motorR.pin_pwm          = PWM_PIN_RIGHT;
  motorR.pin_dir          = DIR_PIN_RIGHT;
  motorR.pin_hall         = HALL_PIN_RIGHT;
  motorR.kp               = 0.7;
  motorR.ki               = 0.10;
  motorR.kd               = 0.02;
  motorR.dir              = true;
  motorR.invert_dir       = false;
  motorR.feedback_raw     = 0;
  motorR.last_pulse_time  = 0;
  motorR.counter          = 0;
  motorR.errSum           = 0;
  motorR.prevErr          = 0;
  motorR.prev_speed_pps   = 0;
  motorR.accel_check_time = 0;
  motorR.prev_pwm         = 0;
  motorR.ff_offset        = 10;
  motorR.startup_pwm_min  = 14;
  motorR.startup_pwm_boost = 32;
  motorR.startup_boost_active = false;
  motorR.startup_boost_until  = 0;
  motorR.enabled          = true;

  // ── Pin setup ──
  pinMode(TEST_PIN, OUTPUT);
  digitalWrite(TEST_PIN, LOW);

  #ifdef MOTOR_LEFT_ENABLED
    pinMode(motorL.pin_pwm, OUTPUT);
    pinMode(motorL.pin_dir, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(HALL_PIN_LEFT), isrLeft, RISING);
    analogWrite(motorL.pin_pwm, 0);
    digitalWrite(motorL.pin_dir, motorL.invert_dir ? !motorL.dir : motorL.dir);
  #endif

  #ifdef MOTOR_RIGHT_ENABLED
    pinMode(motorR.pin_pwm, OUTPUT);
    pinMode(motorR.pin_dir, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(HALL_PIN_RIGHT), isrRight, RISING);
    analogWrite(motorR.pin_pwm, 0);
    digitalWrite(motorR.pin_dir, motorR.invert_dir ? !motorR.dir : motorR.dir);
  #endif

  initSerialCommands();

  Serial.print(F("Firmware: "));
  Serial.println(FIRMWARE_VERSION);
  Serial.println(F("Differential drive ready. SAFETY SYSTEM ACTIVE."));
  Serial.println(F("Units:   U0=pps  U1=RPM  U2=m/s"));
  Serial.println(F("Drive:   V=target  U=unit  O=turn  r=ramp  S=sync  R=reset"));
  Serial.println(F("Debug:   A=advanced(0/1)  M=direct_pwm(0/1)  W=status(0/1)  q/Q=left/right duty %"));
  Serial.println(F("DirInv:  n=left(0/1)  N=right(0/1)"));
  Serial.println(F("Enable:  e=both(0/1)  EL=left(0/1)  ER=right(0/1)"));
  Serial.println(F("PID L:   p/i/d         PID R: P/I/D    Sync: x=Kp y=Ki"));
  Serial.println(F("FF:      f=left_offset  F=right_offset"));
  Serial.println(F("Tuner:   TL=tune Left  TR=tune Right  TQ=abort"));
  Serial.println(F("Info:    Z=firmware version  J=status snapshot"));
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  handleSerialCommandStream();

  // ── Tuner mode — bypass normal control loop ──
  if (tune_state != TUNE_IDLE && tune_state != TUNE_DONE) {
    updateSpeed(*tune_motor);
    runTuner();
    return;
  }

  // ── Emergency stop — hold until manual R reset ──
  if (emergency_active) {
    analogWrite(motorL.pin_pwm, 0);
    analogWrite(motorR.pin_pwm, 0);
    return;
  }

  // ── Compute loop dt ──
  now_f   = millis();
  dt      = (now_f - prvTime) / 1000.0;
  prvTime = now_f;
  if (dt <= 0 || dt > 0.5) return;

  bool advanced_on = (advanced_features_enable != 0.0f);
  bool direct_on   = (direct_pwm_mode != 0.0f);

  // ── Ramp target toward setpoint ──
  float target_pps_desired = convertToPPS(target_user, (int)target_unit);
  if (!direct_on) {
    if (advanced_on && ramp_rate > 0) {
      if      (target_ramped < target_pps_desired) target_ramped = min(target_ramped + ramp_rate, target_pps_desired);
      else if (target_ramped > target_pps_desired) target_ramped = max(target_ramped - ramp_rate, target_pps_desired);
      target_pps = target_ramped;
    } else {
      target_pps    = target_pps_desired;
      target_ramped = target_pps_desired;
    }
  } else {
    target_pps    = 0;
    target_ramped = 0;
  }

  // ── Update speed measurements ──
  #ifdef MOTOR_LEFT_ENABLED
    updateSpeed(motorL);
  #endif
  #ifdef MOTOR_RIGHT_ENABLED
    updateSpeed(motorR);
  #endif

  // ── Reset sync state when stopped — prevents historical error accumulation ──
  if (target_pps < 0.1f || !advanced_on || direct_on) {
    sync_errSum = 0; sync_prevErr = 0; sync_out = 0;
    noInterrupts();
    motorL.counter = 0; motorR.counter = 0;
    interrupts();
  }

  // ── Sync PID ──
  #if defined(MOTOR_LEFT_ENABLED) && defined(MOTOR_RIGHT_ENABLED)
    if (advanced_on && !direct_on && sync_enable > 0 && target_pps > 0) {
      sync_out = runSyncPID(dt);
      float sl = target_pps * SYNC_CLAMP_RATIO;
      sync_out = constrain(sync_out, -sl, sl);
    } else {
      sync_out = 0; sync_errSum = 0; sync_prevErr = 0;
    }
  #else
    sync_out = 0;
  #endif

  // ── Assign per-motor targets ──
  if (!direct_on) {
    motorL.target = max(target_pps - sync_out + turn_offset, 0.0f);
    motorR.target = max(target_pps + sync_out - turn_offset, 0.0f);
  } else {
    motorL.target = 0;
    motorR.target = 0;
  }

  updateLoadFeedforward(advanced_on, direct_on, dt);

  // ── Run control and apply outputs ──
  if (test_disable == 0) {
    if (direct_on) {
      #ifdef MOTOR_LEFT_ENABLED
        if (motorL.enabled) applyDirectDuty(motorL, direct_duty_left);
        else                stopMotor(motorL);
      #endif
      #ifdef MOTOR_RIGHT_ENABLED
        if (motorR.enabled) applyDirectDuty(motorR, direct_duty_right);
        else                stopMotor(motorR);
      #endif
    } else {
      float pidL = 0, pidR = 0;

      #ifdef MOTOR_LEFT_ENABLED
        if (motorL.enabled) pidL = runPID(motorL, motorL.target, dt);
        else                stopMotor(motorL);
      #endif
      #ifdef MOTOR_RIGHT_ENABLED
        if (motorR.enabled) pidR = runPID(motorR, motorR.target, dt);
        else                stopMotor(motorR);
      #endif

      if (advanced_on) {
        unsigned long startup_deadline = millis() + PWM_START_BOOST_MS;
        #ifdef MOTOR_LEFT_ENABLED
          if (motorL.enabled && motorL.target > 0.0f &&
              motorL.speed_pps < STARTUP_SPEED_PPS && motorL.prev_pwm == 0 &&
              !motorL.startup_boost_active) {
            motorL.startup_boost_active = true;
            motorL.startup_boost_until  = startup_deadline;
          }
        #endif
        #ifdef MOTOR_RIGHT_ENABLED
          if (motorR.enabled && motorR.target > 0.0f &&
              motorR.speed_pps < STARTUP_SPEED_PPS && motorR.prev_pwm == 0 &&
              !motorR.startup_boost_active) {
            motorR.startup_boost_active = true;
            motorR.startup_boost_until  = startup_deadline;
          }
        #endif
      }

      bool safeL = true, safeR = true;
      if (advanced_on) {
        #ifdef MOTOR_LEFT_ENABLED
          if (motorL.enabled) safeL = safetyCheck(motorL, dt);
        #endif
        #ifdef MOTOR_RIGHT_ENABLED
          if (motorR.enabled) safeR = safetyCheck(motorR, dt);
        #endif
      }

      if (safeL && safeR) {
        #ifdef MOTOR_LEFT_ENABLED
          if (motorL.enabled) applyPWM(motorL, pidL);
        #endif
        #ifdef MOTOR_RIGHT_ENABLED
          if (motorR.enabled) applyPWM(motorR, pidR);
        #endif
      }
    }
  } else {
    stopMotor(motorL);
    stopMotor(motorR);
  }

  Plotter();
}

// ═══════════════════════════════════════════════════════════════════════════
// COMMON FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

// ── Compute speed from Hall sensor feedback ───────────────────────────────────
void updateSpeed(Motor& m) {
  noInterrupts();
  unsigned long last_p  = m.last_pulse_time;
  unsigned long fb_copy = m.feedback_raw;
  interrupts();

  unsigned long now_us = micros();
  unsigned long age_us = now_us - last_p;
  if (age_us > (unsigned long)MOTOR_TIMEOUT_MS * 1000UL || fb_copy == 0) {
    m.speed_pps *= 0.6f;
    if (m.speed_pps < 0.1f) m.speed_pps = 0.0f;
    m.speed_rpm = (m.speed_pps * 60.0f) / PULSES_PER_REV;
    m.speed_ms  = (m.speed_pps / PULSES_PER_REV) * WHEEL_CIRCUMFERENCE;
  } else {
    unsigned long effective_period_us = max(fb_copy, age_us);
    float pps = 1000000.0f / (float)effective_period_us;

    if (m.target > 0.0f && m.speed_pps < STARTUP_SPEED_PPS && pps > MAX_SAFE_PPS * 1.5f) {
      m.speed_pps = 0; m.speed_rpm = 0; m.speed_ms = 0;
      return;
    }
    if (pps > MAX_HALL_PPS) {
      // Physically impossible speed — floating Hall pin noise
      m.speed_pps = 0; m.speed_rpm = 0; m.speed_ms = 0;
      return;
    }
    m.speed_pps = (m.speed_pps * (1.0f - SPEED_FILTER_ALPHA)) + (pps * SPEED_FILTER_ALPHA);
    m.speed_rpm = (m.speed_pps * 60.0f) / PULSES_PER_REV;
    m.speed_ms  = (m.speed_pps / PULSES_PER_REV) * WHEEL_CIRCUMFERENCE;
  }
}

// ── PID controller ────────────────────────────────────────────────────────────
float runPID(Motor& m, float target, float dt) {
  bool advanced_on = (advanced_features_enable != 0.0f);
  bool near_standstill = (m.speed_pps < STARTUP_SPEED_PPS);

  float err = target - m.speed_pps;
  float P   = m.kp * err;

  if (advanced_on) {
    // Keep building some integral during startup, but more gently.
    // This lets the controller climb toward cruise speed after the startup kick
    // without instantly winding up when the wheel is still stuck.
    bool allow_integral = (!near_standstill || target < 5.0f ||
                           m.startup_boost_active || m.prev_pwm >= m.startup_pwm_min);
    if (allow_integral) {
      float i_scale = near_standstill ? PID_STARTUP_I_SCALE : 1.0f;
      m.errSum = constrain(m.errSum + (err * dt * i_scale),
                           -PID_ERRSUM_LIMIT_ADV, PID_ERRSUM_LIMIT_ADV);
    } else {
      // Decay instead of zeroing, so the controller does not lose all drive memory.
      m.errSum *= 0.92f;
    }
  } else {
    m.errSum = constrain(m.errSum + (err * dt),
                         -PID_ERRSUM_LIMIT_SIMPLE, PID_ERRSUM_LIMIT_SIMPLE);
  }

  float I = m.ki * m.errSum;

  float D = 0;
  if (advanced_on) {
    // Derivative: only active when motor is already moving.
    // Avoids large D spike caused by jump from 0 to first measured speed.
    if (m.speed_pps > 2.0 && m.prevErr != 0 && dt > 0) {
      D = m.kd * (err - m.prevErr) / dt;
      D = constrain(D, -50.0, 50.0);  // Clamp D to prevent spike.
    }
  } else if (dt > 0) {
    D = m.kd * (err - m.prevErr) / dt;
  }
  m.prevErr = err;

  float out = P + I + D;

  if (advanced_on) {
    // Rate-limit PID output change to prevent sudden power jumps.
    out = constrain(out, m.pid_out - 80.0, m.pid_out + 80.0);
  }

  return out;
}

// ── Sync PID — keeps both motor pulse counters equal ─────────────────────────
float runSyncPID(float dt) {
  noInterrupts();
  float err = motorL.counter - motorR.counter;
  interrupts();

  float P      = sync_kp * err;
  sync_errSum  = constrain(sync_errSum + (err * dt), -100.0, 100.0);
  float I      = sync_ki * sync_errSum;
  float D      = (dt > 0) ? sync_kd * (err - sync_prevErr) / dt : 0;
  sync_prevErr = err;
  return P + I + D;
}

// ── Safety checks ─────────────────────────────────────────────────────────────
// Returns false and triggers emergency stop if any limit is exceeded
void updateLoadFeedforward(bool advanced_on, bool direct_on, float dt) {
  if (!advanced_on || direct_on || emergency_active) {
    load_ff_boost = 0.0f;
    return;
  }

  int enabled_count = 0;
  float avg_speed_pps = 0.0f;

  #ifdef MOTOR_LEFT_ENABLED
    if (motorL.enabled) {
      avg_speed_pps += motorL.speed_pps;
      enabled_count++;
    }
  #endif

  #ifdef MOTOR_RIGHT_ENABLED
    if (motorR.enabled) {
      avg_speed_pps += motorR.speed_pps;
      enabled_count++;
    }
  #endif

  if (enabled_count <= 0) {
    load_ff_boost = 0.0f;
    return;
  }

  avg_speed_pps /= enabled_count;

  bool low_target = (target_pps < LOAD_FF_TARGET_MIN_PPS);
  bool sync_unstable = (abs(sync_out) > max(1.5f, target_pps * LOAD_FF_SYNC_GATE));
  bool startup_active = motorL.startup_boost_active || motorR.startup_boost_active;

  if (low_target || startup_active || sync_unstable) {
    load_ff_boost = max(0.0f, load_ff_boost - (LOAD_FF_IDLE_DECAY * dt));
    return;
  }

  float avg_err = constrain(target_pps - avg_speed_pps, -10.0f, 10.0f);
  float adapt_rate = (avg_err >= 0.0f) ? LOAD_FF_UP_RATE : LOAD_FF_DOWN_RATE;
  load_ff_boost = constrain(load_ff_boost + (avg_err * dt * adapt_rate),
                            0.0f, LOAD_FF_MAX);
}

bool safetyCheck(Motor& m, float dt) {
  unsigned long now_ms = millis();
  bool startup_grace = m.startup_boost_active ||
                       (m.startup_boost_until != 0 &&
                        (long)(now_ms - (m.startup_boost_until + STARTUP_SAFETY_GRACE_MS)) < 0) ||
                       (m.target > 0.0f && m.speed_pps < STARTUP_SPEED_PPS && m.prev_pwm >= m.startup_pwm_min);

  // Layer 1: Absolute speed limit
  if (!startup_grace && m.speed_pps > MAX_SAFE_PPS) {
    emergencyStop(F("SPEED LIMIT"));
    return false;
  }

  // Layer 2: Acceleration limit — checked every 200ms window, not every loop.
  // Avoids false triggers from noisy Hall pulses at startup.
  if (!startup_grace && m.speed_pps > 2.0 && m.target > 2.0 && m.prev_speed_pps > 2.0) {
    if (now_ms - m.accel_check_time >= 200) {
      float dt_a  = (now_ms - m.accel_check_time) / 1000.0;
      float accel = abs(m.speed_pps - m.prev_speed_pps) / dt_a;
      if (accel > MAX_SAFE_ACCEL) {
        emergencyStop(F("ACCEL LIMIT"));
        return false;
      }
      m.prev_speed_pps   = m.speed_pps;
      m.accel_check_time = now_ms;
    }
  } else {
    m.prev_speed_pps   = m.speed_pps;
    m.accel_check_time = millis();
  }

  // Layer 3: PID runaway — only checked when motor is already spinning.
  // Prevents false trigger during startup when integral hasn't settled.
  if (m.speed_pps > 2.0 && m.target > 2.0 && abs(m.pid_out) > MAX_PID_OUTPUT) {
    emergencyStop(F("PID RUNAWAY"));
    return false;
  }

  return true;
}

// ── Emergency stop — cuts power and blocks until manual R reset ───────────────
void emergencyStop(const __FlashStringHelper* reason) {
  analogWrite(motorL.pin_pwm, 0);
  analogWrite(motorR.pin_pwm, 0);
  emergency_active = true;
  resetMotor(motorL); resetMotor(motorR);
  sync_errSum = 0; sync_prevErr = 0; sync_out = 0;
  target_ramped = 0;
  Serial.print(F("!!! EMERGENCY STOP: "));
  Serial.print(reason);
  Serial.println(F(" !!! Send R to resume."));
}

// ── Apply PWM with ramp limiting and feedforward offset ───────────────────────
void applyPWM(Motor& m, float pid_out) {
  bool advanced_on = (advanced_features_enable != 0.0f);
  unsigned long now_ms = millis();

  if (m.target < 0.1f) {
    stopMotor(m);
    return;
  }

  int pwm_desired = 0;
  if (advanced_on) {
    bool near_standstill = (m.speed_pps < STARTUP_SPEED_PPS);
    if (near_standstill && m.prev_pwm == 0 && !m.startup_boost_active) {
      m.startup_boost_active = true;
      m.startup_boost_until  = now_ms + PWM_START_BOOST_MS;
    }

    if (!near_standstill || (long)(now_ms - m.startup_boost_until) >= 0) {
      m.startup_boost_active = false;
    }

    // Use a brief startup kick only until the first Hall transitions appear.
    // After that, let PWM fall lower so the PID can hold slower speeds.
    float pwm_base = pid_out + m.ff_offset + load_ff_boost;
    int pwm_floor = 0;
    if (near_standstill) {
      pwm_floor = m.startup_boost_active ? m.startup_pwm_boost : m.startup_pwm_min;
    }
    pwm_desired = (int)constrain(pwm_base, (float)pwm_floor, 255.0f);

    // Ramp limit: max PWM_RAMP_LIMIT change per loop.
    pwm_desired = (int)constrain((float)pwm_desired,
                                 (float)(m.prev_pwm - PWM_RAMP_LIMIT),
                                 (float)(m.prev_pwm + PWM_RAMP_LIMIT));
  } else {
    // Simple mode: raw PID output only, no feedforward and no PWM ramp limiting.
    pwm_desired = (int)constrain(pid_out, 0, 255);
  }

  int pwm_out = constrain(pwm_desired, 0, 255);

  m.prev_pwm   = pwm_out;
  m.duty_cycle = (pwm_out / 255.0) * 100.0;
  m.pid_out    = pid_out;
  bool physical_dir = m.dir;
  if (m.invert_dir) physical_dir = !physical_dir;

  analogWrite(m.pin_pwm, pwm_out);
  digitalWrite(m.pin_dir, physical_dir);
}

// ── Apply direct PWM duty cycle, bypassing PID/target/sync logic ─────────────
void applyDirectDuty(Motor& m, float duty_percent) {
  float duty_abs = constrain(abs(duty_percent), 0.0f, 100.0f);
  int pwm_out    = (int)((duty_abs / 100.0f) * 255.0f);

  m.dir        = (duty_percent >= 0.0f);
  m.prev_pwm   = pwm_out;
  m.duty_cycle = (m.dir ? duty_abs : -duty_abs);
  m.pid_out    = 0;
  m.errSum     = 0;
  m.prevErr    = 0;

  bool physical_dir = m.dir;
  if (m.invert_dir) physical_dir = !physical_dir;

  analogWrite(m.pin_pwm, pwm_out);
  digitalWrite(m.pin_dir, physical_dir);
}

// ── Stop a single motor immediately ──────────────────────────────────────────
void stopMotor(Motor& m) {
  analogWrite(m.pin_pwm, 0);
  m.prev_pwm             = 0;
  m.duty_cycle           = 0;
  m.pid_out              = 0;
  m.startup_boost_active = false;
  m.startup_boost_until  = 0;
}

// ── Full motor state reset ────────────────────────────────────────────────────
void resetMotor(Motor& m) {
  m.errSum           = 0;
  m.prevErr          = 0;
  m.pid_out          = 0;
  m.duty_cycle       = 0;
  m.speed_pps        = 0;
  m.speed_rpm        = 0;
  m.speed_ms         = 0;
  m.prev_speed_pps   = 0;
  m.accel_check_time = 0;
  m.prev_pwm         = 0;
  m.startup_boost_active = false;
  m.startup_boost_until  = 0;
  noInterrupts();
  m.feedback_raw     = 0;
  m.last_pulse_time  = 0;
  m.counter          = 0;
  interrupts();
}

// ── Convert user target value to pps ─────────────────────────────────────────
float convertToPPS(float v, int u) {
  switch (u) {
    case 0: return v;
    case 1: return (v / 60.0) * PULSES_PER_REV;
    case 2: return (v / WHEEL_CIRCUMFERENCE) * PULSES_PER_REV;
    default: return v;
  }
}

// ── Serial plotter output — all strings in Flash to save SRAM ────────────────
void printStatusLine() {
  Serial.print(F("[STATUS] tgt="));  Serial.print(target_pps, 1);
  Serial.print(F("pps ramp="));      Serial.print(target_ramped, 1);
  Serial.print(F(" user="));         Serial.print(target_user, 1);
  Serial.print(F(" mode="));         Serial.print(direct_pwm_mode != 0.0f ? F("DIRECT") : F("PID"));
  Serial.print(F(" adv="));          Serial.print(advanced_features_enable != 0.0f ? F("ON") : F("OFF"));
  Serial.print(F(" estop="));        Serial.print(emergency_active ? F("YES") : F("NO"));
  Serial.print(F(" loadff="));       Serial.print(load_ff_boost, 1);

  #ifdef MOTOR_LEFT_ENABLED
    Serial.print(F(" | L[en="));   Serial.print(motorL.enabled ? F("1") : F("0"));
    Serial.print(F(" spd="));      Serial.print(motorL.speed_pps, 1);
    Serial.print(F("pps rpm="));   Serial.print(motorL.speed_rpm, 1);
    Serial.print(F(" duty="));     Serial.print(motorL.duty_cycle, 1);
    Serial.print(F("% pid="));     Serial.print(motorL.pid_out, 1);
    Serial.print(F(" inv="));      Serial.print(motorL.invert_dir ? F("1") : F("0"));
    Serial.print(F(" cnt="));      Serial.print(motorL.counter, 0);
    Serial.print(F("]"));
  #endif

  #ifdef MOTOR_RIGHT_ENABLED
    Serial.print(F(" | R[en="));   Serial.print(motorR.enabled ? F("1") : F("0"));
    Serial.print(F(" spd="));      Serial.print(motorR.speed_pps, 1);
    Serial.print(F("pps rpm="));   Serial.print(motorR.speed_rpm, 1);
    Serial.print(F(" duty="));     Serial.print(motorR.duty_cycle, 1);
    Serial.print(F("% pid="));     Serial.print(motorR.pid_out, 1);
    Serial.print(F(" inv="));      Serial.print(motorR.invert_dir ? F("1") : F("0"));
    Serial.print(F(" cnt="));      Serial.print(motorR.counter, 0);
    Serial.print(F("]"));
  #endif

  #if defined(MOTOR_LEFT_ENABLED) && defined(MOTOR_RIGHT_ENABLED)
    Serial.print(F(" | sync="));  Serial.print(sync_out, 2);
  #endif

  Serial.println();
}

void Plotter() {
  // Suppress during tuning — tuner prints its own data
  if (tune_state != TUNE_IDLE && tune_state != TUNE_DONE) return;
  if (serial_status_enable == 0.0f) return;

  unsigned long now_ms = millis();
  if (now_ms - last_serial_report_ms < SERIAL_REPORT_INTERVAL_MS) return;
  last_serial_report_ms = now_ms;

  printStatusLine();
}

// ── Register serial commands in lookup table ──────────────────────────────────
void initSerialCommands() {
  Serial.setTimeout(100);
  lut['V'].varPtr = &target_user;       lut['V'].label = "Target_value";
  lut['U'].varPtr = &target_unit;       lut['U'].label = "Target_unit(0=pps,1=RPM,2=m/s)";
  lut['O'].varPtr = &turn_offset;       lut['O'].label = "Turn_offset";
  lut['r'].varPtr = &ramp_rate;         lut['r'].label = "Ramp_rate";
  lut['S'].varPtr = &sync_enable;                lut['S'].label = "Sync_enable";
  lut['A'].varPtr = &advanced_features_enable;  lut['A'].label = "Advanced_features_enable";
  lut['M'].varPtr = &direct_pwm_mode;           lut['M'].label = "Direct_PWM_mode";
  lut['W'].varPtr = &serial_status_enable;      lut['W'].label = "Status_output_enable";
  lut['x'].varPtr = &sync_kp;           lut['x'].label = "Sync_Kp";
  lut['y'].varPtr = &sync_ki;           lut['y'].label = "Sync_Ki";
  lut['p'].varPtr = &motorL.kp;         lut['p'].label = "Left_Kp";
  lut['i'].varPtr = &motorL.ki;         lut['i'].label = "Left_Ki";
  lut['d'].varPtr = &motorL.kd;         lut['d'].label = "Left_Kd";
  lut['P'].varPtr = &motorR.kp;         lut['P'].label = "Right_Kp";
  lut['I'].varPtr = &motorR.ki;         lut['I'].label = "Right_Ki";
  lut['D'].varPtr = &motorR.kd;         lut['D'].label = "Right_Kd";
  lut['f'].varPtr = &motorL.ff_offset;           lut['f'].label = "Left_FF_offset";
  lut['F'].varPtr = &motorR.ff_offset;           lut['F'].label = "Right_FF_offset";
  lut['q'].varPtr = &direct_duty_left;           lut['q'].label = "Left_direct_duty_percent";
  lut['Q'].varPtr = &direct_duty_right;          lut['Q'].label = "Right_direct_duty_percent";
  lut['n'].varPtr = nullptr;                     lut['n'].label = "Left_dir_invert(0/1)";
  lut['N'].varPtr = nullptr;                     lut['N'].label = "Right_dir_invert(0/1)";
  lut['e'].varPtr = &test_disable;      lut['e'].label = "Disable_both(0=run,1=stop)";
  Serial.println(F("Serial command system initialized."));
}

void skipSerialSeparators(char*& cursor) {
  while (*cursor == ' ' || *cursor == '\t' || *cursor == ':' ||
         *cursor == ';' || *cursor == ',' || *cursor == '|') {
    cursor++;
  }
}

bool readSerialFloatArg(char*& cursor, float& value) {
  skipSerialSeparators(cursor);
  char* end_ptr = nullptr;
  value = strtod(cursor, &end_ptr);
  if (end_ptr == cursor) return false;
  cursor = end_ptr;
  return true;
}

bool readSerialIntArg(char*& cursor, int& value) {
  skipSerialSeparators(cursor);
  char* end_ptr = nullptr;
  long parsed = strtol(cursor, &end_ptr, 10);
  if (end_ptr == cursor) return false;
  value  = (int)parsed;
  cursor = end_ptr;
  return true;
}

bool executeSerialCommand(char*& cursor) {
  skipSerialSeparators(cursor);
  if (*cursor == '\0') return false;

  char c1 = *cursor++;

  if (c1 == 'T') {
    char c2 = *cursor;
    if (c2 == '\0') {
      Serial.println(F("Incomplete tuner command. Use TL, TR or TQ."));
      return false;
    }
    cursor++;

    if (c2 == 'L') {
      if (tune_state != TUNE_IDLE && tune_state != TUNE_DONE)
        Serial.println(F("Tuner already active. Send TQ to abort."));
      else
        startTuning(motorL, motorR, "LEFT");
      return true;
    }

    if (c2 == 'R') {
      if (tune_state != TUNE_IDLE && tune_state != TUNE_DONE)
        Serial.println(F("Tuner already active. Send TQ to abort."));
      else
        startTuning(motorR, motorL, "RIGHT");
      return true;
    }

    if (c2 == 'Q') {
      abortTuning();
      return true;
    }

    Serial.print(F("Unknown tuner command: T"));
    Serial.println(c2);
    return true;
  }

  if (c1 == 'E') {
    char c2 = *cursor;
    if (c2 == 'L' || c2 == 'R') {
      cursor++;

      int val = 0;
      if (!readSerialIntArg(cursor, val)) {
        Serial.print(F("Missing value for E"));
        Serial.println(c2);
        return true;
      }

      bool en = (val != 0);
      Motor& m = (c2 == 'L') ? motorL : motorR;
      m.enabled = en;
      if (!en) {
        stopMotor(m);
        resetMotor(m);
      }

      Serial.print(c2 == 'L' ? F("Left") : F("Right"));
      Serial.print(F(" motor -> "));
      Serial.println(en ? F("ENABLED") : F("DISABLED"));
      return true;
    }
  }

  if (c1 == 'n' || c1 == 'N') {
    int val = 0;
    if (!readSerialIntArg(cursor, val)) {
      Serial.print(F("Missing value for "));
      Serial.println(c1);
      return true;
    }

    bool inv = (val != 0);
    Motor& m = (c1 == 'n') ? motorL : motorR;
    m.invert_dir = inv;
    bool physical_dir = m.invert_dir ? !m.dir : m.dir;
    digitalWrite(m.pin_dir, physical_dir);
    Serial.print(c1 == 'n' ? F("Left") : F("Right"));
    Serial.print(F(" direction invert -> "));
    Serial.println(inv ? 1 : 0);
    return true;
  }

  if (c1 == 'R') {
    abortTuning();
    emergency_active          = false;
    motorL.enabled            = true;
    motorR.enabled            = true;
    advanced_features_enable  = 1.0;
    direct_pwm_mode           = 0.0;
    direct_duty_left          = 0.0;
    direct_duty_right         = 0.0;
    load_ff_boost             = 0.0;
    resetMotor(motorL);
    resetMotor(motorR);
    sync_errSum   = 0;
    sync_prevErr  = 0;
    sync_out      = 0;
    target_user   = 0;
    target_pps    = 0;
    target_ramped = 0;
    turn_offset   = 0;
    tune_state    = TUNE_IDLE;
    Serial.println(F("Full reset done. Both motors enabled."));
    return true;
  }

  if (c1 == 'Z') {
    Serial.print(F("Firmware: "));
    Serial.println(FIRMWARE_VERSION);
    return true;
  }

  if (c1 == 'J') {
    printStatusLine();
    return true;
  }

  if (c1 >= 0 && c1 < 128 && lut[(int)c1].varPtr != nullptr) {
    float val = 0.0f;
    if (!readSerialFloatArg(cursor, val)) {
      Serial.print(F("Missing value for "));
      Serial.println(c1);
      return true;
    }

    *(lut[(int)c1].varPtr) = val;

    if (c1 == 'A' && advanced_features_enable == 0.0f) {
      sync_errSum  = 0;
      sync_prevErr = 0;
      sync_out     = 0;
    }

    if (c1 == 'M') {
      if (direct_pwm_mode != 0.0f) {
        target_user   = 0;
        target_pps    = 0;
        target_ramped = 0;
        sync_errSum   = 0;
        sync_prevErr  = 0;
        sync_out      = 0;
        resetMotor(motorL);
        resetMotor(motorR);
      } else {
        direct_duty_left  = 0.0f;
        direct_duty_right = 0.0f;
        stopMotor(motorL);
        stopMotor(motorR);
      }
    }

    Serial.print(lut[(int)c1].label);
    Serial.print(F(" -> "));
    Serial.println(val);
    return true;
  }

  Serial.print(F("Unknown command: "));
  Serial.println(c1);
  return true;
}

void processSerialCommandBuffer() {
  if (serial_command_length <= 0) return;

  serial_command_buffer[serial_command_length] = '\0';
  char* cursor = serial_command_buffer;

  while (executeSerialCommand(cursor)) {
    skipSerialSeparators(cursor);
    if (*cursor == '\0') break;
  }

  serial_command_length = 0;
  serial_command_buffer[0] = '\0';
}

// ── Handle incoming serial commands ──────────────────────────────────────────
// Format: <char><value>    e.g. V60  p2.4  f30
// Two-char commands: TL TR TQ EL ER
// Special: R = full reset
void handleSerialCommands() {
  if (!Serial.available()) return;

  char c1 = Serial.read();

  // ── Two-character commands: T* (tuner) and E* (enable) ──
  if (c1 == 'T' || c1 == 'E') {
    unsigned long t0 = millis();
    while (!Serial.available() && millis() - t0 < 200);
    if (!Serial.available()) return;
    char c2 = Serial.read();

    // ── Tuner commands ──
    if (c1 == 'T') {
      if (c2 == 'L') {
        if (tune_state != TUNE_IDLE && tune_state != TUNE_DONE)
          Serial.println(F("Tuner already active. Send TQ to abort."));
        else
          startTuning(motorL, motorR, "LEFT");
        return;
      }
      if (c2 == 'R') {
        if (tune_state != TUNE_IDLE && tune_state != TUNE_DONE)
          Serial.println(F("Tuner already active. Send TQ to abort."));
        else
          startTuning(motorR, motorL, "RIGHT");
        return;
      }
      if (c2 == 'Q') { abortTuning(); return; }
    }

    // ── Per-motor enable/disable: EL0 EL1 ER0 ER1 ──
    if (c1 == 'E' && (c2 == 'L' || c2 == 'R')) {
      unsigned long t1 = millis();
      while (!Serial.available() && millis() - t1 < 200);
      if (!Serial.available()) return;
      int val = Serial.parseInt();
      bool en = (val != 0);

      Motor& m = (c2 == 'L') ? motorL : motorR;
      m.enabled = en;
      if (!en) { stopMotor(m); resetMotor(m); }

      Serial.print(c2 == 'L' ? F("Left") : F("Right"));
      Serial.print(F(" motor -> "));
      Serial.println(en ? F("ENABLED") : F("DISABLED"));
      return;
    }
    return;
  }

  // ── Direction inversion commands: n0/n1 for left, N0/N1 for right ──
  if (c1 == 'n' || c1 == 'N') {
    while (Serial.peek() == ':' || Serial.peek() == ' ') Serial.read();
    int val = Serial.parseInt();
    bool inv = (val != 0);
    Motor& m = (c1 == 'n') ? motorL : motorR;
    m.invert_dir = inv;
    bool physical_dir = m.invert_dir ? !m.dir : m.dir;
    digitalWrite(m.pin_dir, physical_dir);
    Serial.print(c1 == 'n' ? F("Left") : F("Right"));
    Serial.print(F(" direction invert -> "));
    Serial.println(inv ? 1 : 0);
    return;
  }

  // ── Full reset ──
  if (c1 == 'R') {
    abortTuning();
    emergency_active          = false;
    motorL.enabled            = true;
    motorR.enabled            = true;
    advanced_features_enable  = 1.0;
    direct_pwm_mode           = 0.0;
    direct_duty_left          = 0.0;
    direct_duty_right         = 0.0;
    resetMotor(motorL); resetMotor(motorR);
    sync_errSum   = 0; sync_prevErr = 0; sync_out = 0;
    target_user   = 0; target_pps   = 0;
    target_ramped = 0; turn_offset  = 0;
    tune_state = TUNE_IDLE;
    Serial.println(F("Full reset done. Both motors enabled."));
    return;
  }

  // ── Single-char LUT commands (float value follows) ──
  if (c1 >= 0 && c1 < 128 && lut[(int)c1].varPtr != nullptr) {
    while (Serial.peek() == ':' || Serial.peek() == ' ') Serial.read();
    float val = Serial.parseFloat();
    *(lut[(int)c1].varPtr) = val;

    if (c1 == 'A') {
      if (advanced_features_enable == 0.0f) {
        sync_errSum = 0; sync_prevErr = 0; sync_out = 0;
      }
    }

    if (c1 == 'M') {
      if (direct_pwm_mode != 0.0f) {
        target_user   = 0;
        target_pps    = 0;
        target_ramped = 0;
        sync_errSum   = 0; sync_prevErr = 0; sync_out = 0;
        resetMotor(motorL);
        resetMotor(motorR);
      } else {
        direct_duty_left  = 0.0f;
        direct_duty_right = 0.0f;
        stopMotor(motorL);
        stopMotor(motorR);
      }
    }

    Serial.print(lut[(int)c1].label);
    Serial.print(F(" -> "));
    Serial.println(val);
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// AUTO-TUNER (Ziegler-Nichols closed-loop method)
// ═══════════════════════════════════════════════════════════════════════════

void startTuning(Motor& m, Motor& other, const char* name) {
  tune_motor      = &m;
  tune_other      = &other;
  tune_motor_name = name;

  // Stop the other motor
  stopMotor(other);
  resetMotor(other);

  // Reset all tuner state
  tune_kp_current  = TUNE_KP_START;
  tune_osc_min     = 1e6;  tune_osc_max    = -1e6;
  tune_osc_cross   = 0;
  tune_period_sum  = 0;    tune_period_cnt = 0;
  tune_result_ku   = 0;    tune_result_tu  = 0;
  tune_osc_last    = 0;    tune_last_cross = 0;

  // Start open-loop at fixed PWM
  digitalWrite(m.pin_dir, HIGH);
  analogWrite(m.pin_pwm, TUNE_PWM_INIT);

  tune_phase_start = millis();
  tune_state       = TUNE_PHASE1;

  Serial.println(F("═══════════════════════════════════════"));
  Serial.print(F("  AUTO-TUNER START — Motor: ")); Serial.println(name);
  Serial.print(F("  Phase 1: Open-loop stabilization... PWM=")); Serial.println(TUNE_PWM_INIT);
}

// ─────────────────────────────────────────────────────────────────────────────
void runTuner() {
  unsigned long elapsed = millis() - tune_phase_start;
  float spd = tune_motor->speed_pps;

  switch (tune_state) {

    // ── Phase 1: Open-loop — wait for motor speed to stabilize ──
    case TUNE_PHASE1: {
      Serial.print(F("TUN P1 spd:")); Serial.print(spd, 1);
      Serial.print(F(" pwm:"));       Serial.println(TUNE_PWM_INIT);

      if (elapsed > TUNE_PHASE1_MS) {
        if (spd < 2.0) {
          Serial.println(F("[ERR] Motor not spinning! Check wiring."));
          abortTuning(); return;
        }
        Serial.print(F("[P1 OK] Stable speed: ")); Serial.print(spd, 1); Serial.println(F(" pps"));
        Serial.println(F("  Phase 2: Searching for Ku..."));

        tune_kp_current  = TUNE_KP_START;
        tune_osc_min     = 1e6; tune_osc_max = -1e6;
        tune_osc_cross   = 0;
        tune_osc_last    = spd;
        tune_phase_start = millis();
        tune_state       = TUNE_PHASE2;
      }
      break;
    }

    // ── Phase 2: Increase Kp until sustained oscillation is detected ──
    case TUNE_PHASE2: {
      // P-only controller
      float err     = TUNE_TARGET_PPS - spd;
      float pid_out = tune_kp_current * err;
      analogWrite(tune_motor->pin_pwm, (int)constrain(pid_out, 0, TUNE_PWM_MAX));

      // Track oscillation amplitude
      if (spd > tune_osc_max) tune_osc_max = spd;
      if (spd < tune_osc_min) tune_osc_min = spd;

      // Count zero-crossings relative to target
      bool above_now  = (spd > TUNE_TARGET_PPS);
      bool above_prev = (tune_osc_last > TUNE_TARGET_PPS);
      if (above_now != above_prev) tune_osc_cross++;
      tune_osc_last = spd;

      float amplitude  = tune_osc_max - tune_osc_min;
      bool oscillating = (amplitude > TUNE_OSC_THRESH && tune_osc_cross >= TUNE_MIN_CYCLES * 2);

      Serial.print(F("TUN P2 Kp:")); Serial.print(tune_kp_current, 2);
      Serial.print(F(" spd:"));      Serial.print(spd, 1);
      Serial.print(F(" amp:"));      Serial.print(amplitude, 1);
      Serial.print(F(" cross:"));    Serial.println(tune_osc_cross);

      if (oscillating) {
        tune_result_ku = tune_kp_current;
        Serial.print(F("[P2 OK] Ku = ")); Serial.println(tune_result_ku, 3);
        Serial.println(F("  Phase 3: Measuring Tu..."));

        tune_osc_cross   = 0;
        tune_period_sum  = 0; tune_period_cnt = 0;
        tune_last_cross  = millis();
        tune_osc_last    = spd;
        tune_phase_start = millis();
        tune_state       = TUNE_PHASE3;
        return;
      }

      // After SETTLE_MS without oscillation, increase Kp
      if (elapsed > TUNE_SETTLE_MS) {
        tune_kp_current += TUNE_KP_STEP;
        tune_osc_min = 1e6; tune_osc_max = -1e6; tune_osc_cross = 0;
        tune_phase_start = millis();
        Serial.print(F("  Kp increased to: ")); Serial.println(tune_kp_current, 2);

        if (tune_kp_current > TUNE_KP_MAX) {
          Serial.println(F("[ERR] Max Kp reached without oscillation."));
          Serial.println(F("  Try increasing TUNE_PWM_MAX or TUNE_TARGET_PPS."));
          abortTuning();
        }
      }
      break;
    }

    // ── Phase 3: Measure oscillation period at Ku ──
    case TUNE_PHASE3: {
      float err     = TUNE_TARGET_PPS - spd;
      float pid_out = tune_result_ku * err;
      analogWrite(tune_motor->pin_pwm, (int)constrain(pid_out, 0, TUNE_PWM_MAX));

      // Measure half-periods via falling zero-crossings through target
      bool above_now  = (spd > TUNE_TARGET_PPS);
      bool above_prev = (tune_osc_last > TUNE_TARGET_PPS);
      if (above_prev && !above_now) {
        unsigned long now_ms = millis();
        if (tune_last_cross > 0) {
          tune_period_sum += (now_ms - tune_last_cross) * 2UL;
          tune_period_cnt++;
        }
        tune_last_cross = now_ms;
      }
      tune_osc_last = spd;

      Serial.print(F("TUN P3 Ku:")); Serial.print(tune_result_ku, 2);
      Serial.print(F(" spd:"));      Serial.print(spd, 1);
      Serial.print(F(" cycles:"));   Serial.println(tune_period_cnt);

      if (tune_period_cnt >= TUNE_MIN_CYCLES) {
        tune_result_tu = (tune_period_sum / tune_period_cnt) / 1000.0;
        tunerComputeAndPrint();
        return;
      }

      if (elapsed > TUNE_OBSERVE_MS) {
        Serial.println(F("[ERR] Not enough cycles. Oscillation may be unstable."));
        abortTuning();
      }
      break;
    }

    default: break;
  }
}

// ── Compute Z-N coefficients, apply to motor, and print results ───────────────
void tunerComputeAndPrint() {
  stopMotor(*tune_motor);

  // Ziegler-Nichols PID formulas:
  //   Kp = 0.6 * Ku
  //   Ti = 0.5 * Tu  =>  Ki = Kp / Ti
  //   Td = 0.125 * Tu =>  Kd = Kp * Td
  float kp_res = 0.6   * tune_result_ku;
  float ki_res = kp_res / (0.5   * tune_result_tu);
  float kd_res = kp_res * 0.125  * tune_result_tu;

  Serial.println(F("\n═══════════════════════════════════════"));
  Serial.println(F("  AUTO-TUNER RESULTS"));
  Serial.println(F("═══════════════════════════════════════"));
  Serial.print(F("  Motor: ")); Serial.println(tune_motor_name);
  Serial.print(F("  Ku:    ")); Serial.println(tune_result_ku, 4);
  Serial.print(F("  Tu:    ")); Serial.print(tune_result_tu, 4); Serial.println(F(" s"));
  Serial.println(F("───────────────────────────────────────"));
  Serial.print(F("  Kp = ")); Serial.println(kp_res, 4);
  Serial.print(F("  Ki = ")); Serial.println(ki_res, 4);
  Serial.print(F("  Kd = ")); Serial.println(kd_res, 4);
  Serial.println(F("───────────────────────────────────────"));

  // Apply automatically to the tuned motor
  tune_motor->kp      = kp_res;
  tune_motor->ki      = ki_res;
  tune_motor->kd      = kd_res;
  tune_motor->errSum  = 0;
  tune_motor->prevErr = 0;

  bool isLeft = (tune_motor == &motorL);
  Serial.println(F("  Applied automatically. Equivalent commands:"));
  Serial.print(F("  ")); Serial.print(isLeft ? 'p' : 'P'); Serial.println(kp_res, 3);
  Serial.print(F("  ")); Serial.print(isLeft ? 'i' : 'I'); Serial.println(ki_res, 3);
  Serial.print(F("  ")); Serial.print(isLeft ? 'd' : 'D'); Serial.println(kd_res, 3);
  Serial.println(F("───────────────────────────────────────"));
  Serial.println(F("  NOTE: Z-N can be aggressive."));
  Serial.println(F("  If oscillating, try reduced values:"));
  Serial.print(F("  ")); Serial.print(isLeft ? 'p' : 'P'); Serial.println(kp_res * 0.75, 3);
  Serial.print(F("  ")); Serial.print(isLeft ? 'i' : 'I'); Serial.println(ki_res * 0.5,  3);
  Serial.print(F("  ")); Serial.print(isLeft ? 'd' : 'D'); Serial.println(kd_res * 0.5,  3);
  Serial.println(F("═══════════════════════════════════════"));
  Serial.println(F("  Send R to resume normal drive."));

  tune_state = TUNE_DONE;
}

// ── Abort tuning safely ───────────────────────────────────────────────────────
void abortTuning() {
  bool was_active = (tune_state != TUNE_IDLE && tune_state != TUNE_DONE);
  if (tune_motor) stopMotor(*tune_motor);
  tune_motor      = nullptr;
  tune_other      = nullptr;
  tune_motor_name = nullptr;
  tune_state = TUNE_IDLE;
  if (was_active) {
    Serial.println(F("[TUNER ABORTED] Send TL/TR to retry or R to reset."));
  }
}

void handleSerialCommandStream() {
  while (Serial.available()) {
    char ch = Serial.read();
    serial_last_byte_ms = millis();

    if (ch == '\r' || ch == '\n') {
      processSerialCommandBuffer();
      continue;
    }

    if (serial_command_length >= SERIAL_COMMAND_BUFFER_LEN - 1) {
      Serial.println(F("Serial command too long. Buffer cleared."));
      serial_command_length = 0;
      serial_command_buffer[0] = '\0';
      continue;
    }

    serial_command_buffer[serial_command_length++] = ch;
  }

  if (serial_command_length > 0 &&
      millis() - serial_last_byte_ms >= SERIAL_COMMAND_IDLE_MS) {
    processSerialCommandBuffer();
  }
}
