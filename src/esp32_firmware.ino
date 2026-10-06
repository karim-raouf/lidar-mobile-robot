/*
 * ESP32 closed-loop velocity controller for 2x GA25-370 (encoder) motors on an L298N.
 *
 * Requires arduino-esp32 core 3.x (ESP-IDF 5), no external libraries.
 * Encoders are decoded in hardware by the PCNT peripheral (x4 quadrature, glitch filtered).
 *
 * Serial protocol (ASCII lines, '\n' terminated) - see serial_driver.hpp
 *   in : "V <left_rad_s> <right_rad_s>"   velocity setpoints (feeds the watchdog)
 *        "P <kp> <ki> <kd>"               set PID gains
 *        "R"                              reset encoders
 *   out: "S <pos_l> <pos_r> <vel_l> <vel_r>"  every 20 ms (rad, rad, rad/s, rad/s)
 *
 * Do NOT print debug text on Serial other than lines starting with 'A ' (they are ignored
 * by the host), or you will pollute the link.
 *
 * Wiring (defaults below, change the pin constants if needed)
 *   L298N ENA -> GPIO25 (PWM)   IN1 -> GPIO26   IN2 -> GPIO27      (left motor)
 *   L298N ENB -> GPIO23 (PWM)   IN3 -> GPIO32   IN4 -> GPIO33      (right motor)
 *   Left  encoder  A -> GPIO18, B -> GPIO19
 *   Right encoder  A -> GPIO16, B -> GPIO17
 *   Encoder VCC -> ESP32 3V3 (NOT 5V, ESP32 inputs are not 5V tolerant), GND -> GND
 *   L298N GND must be connected to ESP32 GND. Remove the ENA/ENB jumpers on the L298N.
 */

#include <Arduino.h>
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"

#if ESP_ARDUINO_VERSION_MAJOR < 3
#error "This firmware requires arduino-esp32 core 3.x"
#endif

// =====================================================================================
//                                    USER CONFIG
// =====================================================================================
constexpr uint32_t SERIAL_BAUD = 115200;

// ---- L298N pins ----
constexpr int PIN_L_EN = 25, PIN_L_IN1 = 26, PIN_L_IN2 = 27;
constexpr int PIN_R_EN = 23, PIN_R_IN1 = 32, PIN_R_IN2 = 33;

// ---- encoder pins ----
constexpr int PIN_L_ENC_A = 18, PIN_L_ENC_B = 19;
constexpr int PIN_R_ENC_A = 16, PIN_R_ENC_B = 17;

// ---- direction fixes: the two motors are mirrored, so one side usually needs both flipped.
// Rule: positive velocity must make the robot drive FORWARD and positive encoder counts.
constexpr bool   MOTOR_INVERT[2] = {false, true};
constexpr int8_t ENC_SIGN[2]     = {1, -1};

// ---- encoder / gearbox ----
// GA25-370: usually 11 pulses per motor-shaft revolution per channel.
// Check your gearbox label for the ratio (e.g. 34, 60, 90, 120 ...).
constexpr float ENCODER_PPR = 11.0f;
constexpr float GEAR_RATIO  = 34.0f;
constexpr float COUNTS_PER_REV = ENCODER_PPR * 4.0f * GEAR_RATIO;  // x4 quadrature decoding
constexpr float RAD_PER_COUNT  = TWO_PI / COUNTS_PER_REV;

// ---- control ----
constexpr uint32_t CONTROL_PERIOD_US = 4000;  // 250 Hz PID
constexpr uint32_t PUBLISH_PERIOD_MS = 10;     // 100 Hz feedback
constexpr uint32_t CMD_TIMEOUT_MS    = 500;     // stop motors if no "V" command arrives
constexpr float    VEL_FILTER_ALPHA  = 0.25f;   // 0..1, lower = smoother but laggier
constexpr float    MAX_WHEEL_RAD_S   = 16.0f;   // no-load speed at your supply voltage;
constexpr bool     USE_FEEDFORWARD   = true;    // used for feed-forward and command clamp


// ---- PWM / L298N ----
constexpr uint32_t PWM_FREQ_HZ = 10000;
constexpr uint8_t  PWM_BITS    = 10;
constexpr uint32_t PWM_MAX     = (1u << PWM_BITS) - 1;
constexpr float    MIN_DUTY    = 0.0f;   // dead-band compensation (0.0-0.3), raise if motors
                                         // hum without moving at low commands
constexpr bool     BRAKE_ON_STOP = true; // true: short windings when stopped, false: coast

// =====================================================================================
//                                    ENCODERS
// =====================================================================================
constexpr uint32_t ENC_GLITCH_NS = 1000;  // ignore pulses shorter than this (noise)

struct Encoder {
  int pin_a, pin_b;
  int8_t sign;
  pcnt_unit_handle_t unit = nullptr;
};

Encoder enc[2] = {
  {PIN_L_ENC_A, PIN_L_ENC_B, ENC_SIGN[0]},
  {PIN_R_ENC_A, PIN_R_ENC_B, ENC_SIGN[1]},
};

/// One PCNT unit per encoder, two channels -> counts all 4 edges per pulse.
/// Direction: A rising while B is low counts down, B rising while A is low counts up.
void encoderInit(Encoder &e) {
  pcnt_unit_config_t unit_cfg = {};
  unit_cfg.low_limit = INT16_MIN;
  unit_cfg.high_limit = INT16_MAX;
  unit_cfg.flags.accum_count = 1;  // driver extends the 16-bit hardware counter to int
  ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &e.unit));

  pcnt_glitch_filter_config_t filter_cfg = {};
  filter_cfg.max_glitch_ns = ENC_GLITCH_NS;
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(e.unit, &filter_cfg));

  pcnt_chan_config_t a_cfg = {};
  a_cfg.edge_gpio_num = e.pin_a;
  a_cfg.level_gpio_num = e.pin_b;
  pcnt_chan_config_t b_cfg = {};
  b_cfg.edge_gpio_num = e.pin_b;
  b_cfg.level_gpio_num = e.pin_a;
  pcnt_channel_handle_t ch_a = nullptr, ch_b = nullptr;
  ESP_ERROR_CHECK(pcnt_new_channel(e.unit, &a_cfg, &ch_a));
  ESP_ERROR_CHECK(pcnt_new_channel(e.unit, &b_cfg, &ch_b));

  // edge action: (rising, falling); level action: (control high, control low)
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
    ch_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(
    ch_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
    ch_b, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(
    ch_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

  // accum_count needs watch points at the limits to catch the 16-bit overflow
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(e.unit, unit_cfg.low_limit));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(e.unit, unit_cfg.high_limit));

  // encoder outputs are open-collector on many boards; only touch the pull bits so the
  // PCNT routing set up above is left alone
  gpio_pullup_en(static_cast<gpio_num_t>(e.pin_a));
  gpio_pullup_en(static_cast<gpio_num_t>(e.pin_b));

  ESP_ERROR_CHECK(pcnt_unit_enable(e.unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(e.unit));
  ESP_ERROR_CHECK(pcnt_unit_start(e.unit));
}

int32_t encoderCount(const Encoder &e) {
  int count = 0;
  pcnt_unit_get_count(e.unit, &count);
  return e.sign * count;
}

// =====================================================================================
//                                     MOTORS
// =====================================================================================
struct MotorPins { int en, in1, in2; };
constexpr MotorPins MOTOR[2] = {{PIN_L_EN, PIN_L_IN1, PIN_L_IN2}, {PIN_R_EN, PIN_R_IN1, PIN_R_IN2}};

void pwmInit(int i) { ledcAttach(MOTOR[i].en, PWM_FREQ_HZ, PWM_BITS); }

void pwmWrite(int i, uint32_t duty) { ledcWrite(MOTOR[i].en, duty); }

/// u in [-1, 1]
void setMotor(int i, float u) {
  u = constrain(u, -1.0f, 1.0f);
  if (MOTOR_INVERT[i]) u = -u;
  const float mag = fabsf(u);

  if (mag < 0.001f) {
    if (BRAKE_ON_STOP) {
      digitalWrite(MOTOR[i].in1, HIGH);
      digitalWrite(MOTOR[i].in2, HIGH);
      pwmWrite(i, PWM_MAX);
    } else {
      digitalWrite(MOTOR[i].in1, LOW);
      digitalWrite(MOTOR[i].in2, LOW);
      pwmWrite(i, 0);
    }
    return;
  }

  const float duty = MIN_DUTY + mag * (1.0f - MIN_DUTY);
  digitalWrite(MOTOR[i].in1, u > 0.0f ? HIGH : LOW);
  digitalWrite(MOTOR[i].in2, u > 0.0f ? LOW : HIGH);
  pwmWrite(i, (uint32_t)(duty * PWM_MAX));
}

// =====================================================================================
//                                  CONTROL LOOP
// =====================================================================================
// Output is normalised duty -1..1, error in rad/s; override at runtime with the "P" command.
struct PidGains {
  float kp = 0.04f;
  float ki = 0.25f;
  float kd = 0.0f;
};
PidGains pid;

struct Wheel {
  float target = 0.0f;   // rad/s setpoint
  float pos = 0.0f;      // rad
  float vel = 0.0f;      // rad/s (filtered)
  float integral = 0.0f; // already multiplied by ki (output units)
  int32_t prev_count = 0;
};
Wheel wheel[2];

void controlStep(float dt) {
  for (int i = 0; i < 2; ++i) {
    Wheel &w = wheel[i];

    // --- measurement ---
    const int32_t c = encoderCount(enc[i]);
    const int32_t dc = c - w.prev_count;
    w.prev_count = c;
    const float raw_vel = dc * RAD_PER_COUNT / dt;
    const float prev_vel = w.vel;
    w.vel += VEL_FILTER_ALPHA * (raw_vel - w.vel);
    w.pos = c * RAD_PER_COUNT;

    // --- PID (+ feed-forward), derivative on measurement ---
    float u = 0.0f;
    if (w.target == 0.0f) {
      w.integral = 0.0f;  // stopped: brake/coast, no integrator build-up
    } else {
      const float err = w.target - w.vel;
      const float p = pid.kp * err;
      const float d = -pid.kd * (w.vel - prev_vel) / dt;
      const float ff = USE_FEEDFORWARD ? (w.target / MAX_WHEEL_RAD_S) : 0.0f;
      const float cand = w.integral + pid.ki * err * dt;

      const float u_raw = ff + p + cand + d;
      u = constrain(u_raw, -1.0f, 1.0f);

      // anti-windup: only integrate when not saturated, or when the error pulls us back
      const bool saturated = (u_raw != u);
      if (!saturated || (u_raw > 1.0f && err < 0.0f) || (u_raw < -1.0f && err > 0.0f)) {
        w.integral = constrain(cand, -1.0f, 1.0f);
      }
    }
    setMotor(i, u);
  }
}

// =====================================================================================
//                                 SERIAL PROTOCOL
// =====================================================================================
struct SerialRx {
  char buf[96];
  size_t len = 0;
  bool overflow = false;  // true: discard bytes until the end of the current line
};
SerialRx rx;

struct Timing {
  uint32_t last_ctrl_us = 0;
  uint32_t last_pub_ms = 0;
  uint32_t last_cmd_ms = 0;
};
Timing timing;

void resetEncoders() {
  for (int i = 0; i < 2; ++i) {
    pcnt_unit_clear_count(enc[i].unit);  // also clears the accumulated overflow value
    const float target = wheel[i].target;  // keep the setpoint, clear the rest
    wheel[i] = Wheel{};
    wheel[i].target = target;
  }
}

void handleLine(char *line) {
  switch (line[0]) {
    case 'V': {
      float l, r;
      if (sscanf(line + 1, "%f %f", &l, &r) == 2 && isfinite(l) && isfinite(r)) {
        wheel[0].target = constrain(l, -MAX_WHEEL_RAD_S, MAX_WHEEL_RAD_S);
        wheel[1].target = constrain(r, -MAX_WHEEL_RAD_S, MAX_WHEEL_RAD_S);
        timing.last_cmd_ms = millis();
      }
      break;
    }
    case 'P': {
      float p, i, d;
      if (sscanf(line + 1, "%f %f %f", &p, &i, &d) == 3 && isfinite(p) && isfinite(i) && isfinite(d)) {
        pid = {p, i, d};
        wheel[0].integral = wheel[1].integral = 0.0f;
        Serial.printf("A PID %.5f %.5f %.5f\n", pid.kp, pid.ki, pid.kd);
      }
      break;
    }
    case 'R':
      resetEncoders();
      Serial.println("A RESET");
      break;
    default:
      break;
  }
}

void readSerial() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rx.len > 0 && !rx.overflow) {
        rx.buf[rx.len] = '\0';
        handleLine(rx.buf);
      }
      rx.len = 0;
      rx.overflow = false;
    } else if (rx.overflow) {
      // still inside an over-long line: drop it entirely
    } else if (rx.len < sizeof(rx.buf) - 1) {
      rx.buf[rx.len++] = c;
    } else {
      rx.overflow = true;
    }
  }
}

void publishState() {
  char out[96];
  const int n = snprintf(out, sizeof(out), "S %.4f %.4f %.4f %.4f\n",
                         wheel[0].pos, wheel[1].pos, wheel[0].vel, wheel[1].vel);
  if (n > 0) Serial.write((const uint8_t *)out, (size_t)n);
}

// =====================================================================================
//                                  ARDUINO ENTRY
// =====================================================================================
void setup() {
  Serial.begin(SERIAL_BAUD);

  for (int i = 0; i < 2; ++i) {
    pinMode(MOTOR[i].in1, OUTPUT);
    pinMode(MOTOR[i].in2, OUTPUT);
    pwmInit(i);
    setMotor(i, 0.0f);
  }

  for (Encoder &e : enc) {
    encoderInit(e);
  }

  timing.last_ctrl_us = micros();
  timing.last_pub_ms = millis();
  timing.last_cmd_ms = millis();
  Serial.println("A READY");
}

void loop() {
  readSerial();

  const uint32_t nowUs = micros();
  if (nowUs - timing.last_ctrl_us >= CONTROL_PERIOD_US) {
    const float dt = (nowUs - timing.last_ctrl_us) * 1e-6f;
    timing.last_ctrl_us = nowUs;

    // watchdog: host went silent -> stop
    if (millis() - timing.last_cmd_ms > CMD_TIMEOUT_MS) {
      wheel[0].target = 0.0f;
      wheel[1].target = 0.0f;
    }
    controlStep(dt);
  }

  const uint32_t nowMs = millis();
  if (nowMs - timing.last_pub_ms >= PUBLISH_PERIOD_MS) {
    timing.last_pub_ms = nowMs;
    publishState();
  }
}