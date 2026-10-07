/*
 * =====================================================================================
 *  ESP32 WHEEL SPEED CONTROLLER
 * =====================================================================================
 *
 * WHAT THIS PROGRAM DOES
 *   The robot has two wheels. Each wheel is driven by a GA25-370 DC motor that has a
 *   built-in encoder (a sensor that tells us how far the motor shaft has turned).
 *   Both motors are powered through one L298N motor driver board.
 *
 *   The computer (ROS 2) sends us the speed it wants for each wheel. This program:
 *     1. reads the encoders to measure how fast each wheel is ACTUALLY turning,
 *     2. uses a PID controller to adjust the motor power until actual speed = wanted speed,
 *     3. sends the measured wheel position and speed back to the computer.
 *
 *   Requires arduino-esp32 core 3.x (ESP-IDF 5). No external libraries.
 *   Encoder pulses are counted by the ESP32's hardware pulse counter (PCNT), so we never
 *   miss a pulse even when the CPU is busy.
 *
 * SERIAL PROTOCOL (text lines ending with '\n') - must match serial_driver.hpp on the PC
 *   Computer -> ESP32:
 *     "V <left_speed> <right_speed>"   wanted wheel speeds in rad/s
 *                                      (also tells us the computer is still alive)
 *     "P <kp> <ki> <kd>"               change the PID gains while running
 *     "R"                              reset the encoder counts to zero
 *   ESP32 -> Computer (every 10 ms):
 *     "S <left_pos> <right_pos> <left_speed> <right_speed>"   (rad, rad, rad/s, rad/s)
 *
 *   IMPORTANT: Do NOT print anything else on Serial unless the line starts with "A "
 *   (the computer ignores those lines). Any other text will confuse the computer.
 *
 * WIRING
 *   Left motor : L298N ENA -> GPIO25 (speed/PWM)  IN1 -> GPIO26  IN2 -> GPIO27
 *   Right motor: L298N ENB -> GPIO23 (speed/PWM)  IN3 -> GPIO32  IN4 -> GPIO33
 *   Left  encoder: A -> GPIO18, B -> GPIO19
 *   Right encoder: A -> GPIO16, B -> GPIO17
 *   Encoder VCC -> ESP32 3V3 (NOT 5V, the ESP32 pins are not 5V tolerant), GND -> GND
 *   L298N GND must be connected to ESP32 GND. Remove the ENA/ENB jumpers on the L298N.
 */

#include <Arduino.h>
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"

#if ESP_ARDUINO_VERSION_MAJOR < 3
#error "This firmware requires arduino-esp32 core 3.x"
#endif

// =====================================================================================
//                                     SETTINGS
//               (everything you might want to change is in this section)
// =====================================================================================

// We use index 0 for the left wheel and index 1 for the right wheel in every array.
constexpr int LEFT       = 0;
constexpr int RIGHT      = 1;
constexpr int NUM_WHEELS = 2;

constexpr uint32_t SERIAL_BAUD_RATE = 115200;

// ---- Motor driver (L298N) pins ----
// PWM pin       = sets how much power the motor gets (0% .. 100%)
// direction pins = the two inputs that choose forward / backward / brake
constexpr int LEFT_MOTOR_PWM_PIN    = 25;  // ENA
constexpr int LEFT_MOTOR_DIR_PIN_1  = 26;  // IN1
constexpr int LEFT_MOTOR_DIR_PIN_2  = 27;  // IN2
constexpr int RIGHT_MOTOR_PWM_PIN   = 23;  // ENB
constexpr int RIGHT_MOTOR_DIR_PIN_1 = 32;  // IN3
constexpr int RIGHT_MOTOR_DIR_PIN_2 = 33;  // IN4

// ---- Encoder pins (each encoder has two signal wires: A and B) ----
constexpr int LEFT_ENCODER_PIN_A  = 18;
constexpr int LEFT_ENCODER_PIN_B  = 19;
constexpr int RIGHT_ENCODER_PIN_A = 16;
constexpr int RIGHT_ENCODER_PIN_B = 17;

// ---- Direction fixes ----
// The two motors are mounted mirrored, so one side usually needs to be flipped.
// Rule: a POSITIVE speed command must drive the robot FORWARD and make the encoder
// count go UP. If a wheel spins the wrong way or counts backwards, flip it here.
constexpr bool   MOTOR_IS_REVERSED[NUM_WHEELS] = {false, true};  // {left, right}
constexpr int8_t ENCODER_DIRECTION[NUM_WHEELS] = {1, -1};        // {left, right}: +1 or -1

// ---- Encoder and gearbox ----
// The encoder sits on the motor shaft (before the gearbox), so the wheel turns
// GEARBOX_RATIO times slower than the shaft the encoder is measuring.
constexpr float ENCODER_PULSES_PER_MOTOR_REV = 11.0f;  // GA25-370: usually 11 per channel
constexpr float GEARBOX_RATIO                = 34.0f;  // see your gearbox label (34, 60, 90...)
// We count every rising AND falling edge on BOTH channels A and B -> 4 counts per pulse.
constexpr float ENCODER_COUNTS_PER_WHEEL_REV = ENCODER_PULSES_PER_MOTOR_REV * 4.0f * GEARBOX_RATIO;
constexpr float RADIANS_PER_ENCODER_COUNT    = TWO_PI / ENCODER_COUNTS_PER_WHEEL_REV;

// ---- Timing ----
constexpr uint32_t CONTROL_LOOP_PERIOD_US  = 4000;  // run the PID every 4 ms (250 times/s)
constexpr uint32_t STATE_PUBLISH_PERIOD_MS = 10;    // send "S" line every 10 ms (100 times/s)
constexpr uint32_t COMMAND_TIMEOUT_MS      = 200;   // safety: stop the motors if no "V"
                                                    // command arrives for this long

// ---- Speed control ----
// Measured speed is noisy, so we smooth it. 0..1: lower = smoother but reacts slower.
constexpr float VELOCITY_SMOOTHING = 0.25f;
// Top wheel speed (rad/s) the motor reaches at full power with your battery voltage.
// Speed commands are limited to this, and it is also used by the feed-forward below.
constexpr float MAX_WHEEL_SPEED_RAD_S = 16.0f;
// Feed-forward = a first guess of the motor power from the wanted speed
// (wanted / max speed). The PID then only has to correct the small remaining error.
constexpr bool USE_FEEDFORWARD = true;

// ---- Motor power (PWM) ----
constexpr uint32_t PWM_FREQUENCY_HZ    = 10000;
constexpr uint8_t  PWM_RESOLUTION_BITS = 10;
constexpr uint32_t PWM_MAX_VALUE       = (1u << PWM_RESOLUTION_BITS) - 1;  // = 1023 = 100% power
// Minimum motor power (0.0 - 0.3). Small powers are often too weak to turn the motor.
// Raise this if the motors hum without moving when asked to go slowly.
constexpr float MIN_MOTOR_POWER = 0.0f;
// What to do when the wanted speed is zero:
//   true  = brake (motor is actively held still)
//   false = coast (motor is switched off and rolls freely)
constexpr bool BRAKE_WHEN_STOPPED = true;

// ---- Encoder noise filter ----
constexpr uint32_t ENCODER_NOISE_FILTER_NS = 1000;  // ignore pulses shorter than 1 us (noise)

// =====================================================================================
//                                     ENCODERS
// =====================================================================================

struct Encoder {
  int    pinA;
  int    pinB;
  int8_t direction;                               // +1 or -1, see ENCODER_DIRECTION
  pcnt_unit_handle_t hardwareCounter = nullptr;   // the ESP32 pulse counter we use
};

Encoder encoders[NUM_WHEELS] = {
  {LEFT_ENCODER_PIN_A,  LEFT_ENCODER_PIN_B,  ENCODER_DIRECTION[LEFT]},
  {RIGHT_ENCODER_PIN_A, RIGHT_ENCODER_PIN_B, ENCODER_DIRECTION[RIGHT]},
};

/// Sets up one hardware pulse counter for one encoder.
/// It watches both wires A and B and counts every edge (4 counts per encoder pulse).
/// Which wire changes first tells us the turning direction, so the count goes up when
/// turning one way and down when turning the other way.
void setupEncoder(Encoder &encoder) {
  // 1. Create the counter. The hardware counter is only 16 bits (-32768..32767);
  //    "accum_count" makes the driver keep counting past those limits for us.
  pcnt_unit_config_t counterConfig = {};
  counterConfig.low_limit  = INT16_MIN;
  counterConfig.high_limit = INT16_MAX;
  counterConfig.flags.accum_count = 1;
  ESP_ERROR_CHECK(pcnt_new_unit(&counterConfig, &encoder.hardwareCounter));

  // 2. Ignore very short spikes on the wires (electrical noise).
  pcnt_glitch_filter_config_t noiseFilterConfig = {};
  noiseFilterConfig.max_glitch_ns = ENCODER_NOISE_FILTER_NS;
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(encoder.hardwareCounter, &noiseFilterConfig));

  // 3. Two channels: one counts edges on wire A (looking at B to know the direction),
  //    the other counts edges on wire B (looking at A to know the direction).
  pcnt_chan_config_t channelAConfig = {};
  channelAConfig.edge_gpio_num  = encoder.pinA;
  channelAConfig.level_gpio_num = encoder.pinB;
  pcnt_chan_config_t channelBConfig = {};
  channelBConfig.edge_gpio_num  = encoder.pinB;
  channelBConfig.level_gpio_num = encoder.pinA;
  pcnt_channel_handle_t channelA = nullptr;
  pcnt_channel_handle_t channelB = nullptr;
  ESP_ERROR_CHECK(pcnt_new_channel(encoder.hardwareCounter, &channelAConfig, &channelA));
  ESP_ERROR_CHECK(pcnt_new_channel(encoder.hardwareCounter, &channelBConfig, &channelB));

  // 4. Counting rules (this is standard "x4 quadrature decoding"):
  //    edge action  = what to do on a (rising edge, falling edge)
  //    level action = what to do when the other wire is (HIGH, LOW): keep or flip direction
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
    channelA, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(
    channelA, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
    channelB, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(
    channelB, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

  // 5. "accum_count" needs these watch points to notice when the 16-bit counter overflows.
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(encoder.hardwareCounter, counterConfig.low_limit));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(encoder.hardwareCounter, counterConfig.high_limit));

  // 6. Many encoders can only pull the wire LOW, so turn on the ESP32's internal pull-up
  //    resistors. We only change the pull-up setting so the counter wiring above stays intact.
  gpio_pullup_en(static_cast<gpio_num_t>(encoder.pinA));
  gpio_pullup_en(static_cast<gpio_num_t>(encoder.pinB));

  // 7. Start counting from zero.
  ESP_ERROR_CHECK(pcnt_unit_enable(encoder.hardwareCounter));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(encoder.hardwareCounter));
  ESP_ERROR_CHECK(pcnt_unit_start(encoder.hardwareCounter));
}

/// Total encoder counts since start (or since the last reset), with the direction fix applied.
int32_t readEncoderCount(const Encoder &encoder) {
  int count = 0;
  pcnt_unit_get_count(encoder.hardwareCounter, &count);
  return encoder.direction * count;
}

// =====================================================================================
//                                      MOTORS
// =====================================================================================

struct MotorPins {
  int pwmPin;   // power (speed) pin
  int dirPin1;  // direction pin 1
  int dirPin2;  // direction pin 2
};

constexpr MotorPins motorPins[NUM_WHEELS] = {
  {LEFT_MOTOR_PWM_PIN,  LEFT_MOTOR_DIR_PIN_1,  LEFT_MOTOR_DIR_PIN_2},
  {RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_DIR_PIN_1, RIGHT_MOTOR_DIR_PIN_2},
};

void setupMotor(int wheel) {
  pinMode(motorPins[wheel].dirPin1, OUTPUT);
  pinMode(motorPins[wheel].dirPin2, OUTPUT);
  ledcAttach(motorPins[wheel].pwmPin, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
}

/// Sets the motor power.
///   power = +1.0  -> full power forward
///   power =  0.0  -> stop (brake or coast, see BRAKE_WHEN_STOPPED)
///   power = -1.0  -> full power backward
///
/// How the L298N works:
///   dirPin1 HIGH, dirPin2 LOW  -> spin one way
///   dirPin1 LOW,  dirPin2 HIGH -> spin the other way
///   both HIGH                  -> brake
///   both LOW                   -> coast
///   the PWM pin sets how strong (0 .. PWM_MAX_VALUE)
void setMotorPower(int wheel, float power) {
  power = constrain(power, -1.0f, 1.0f);
  if (MOTOR_IS_REVERSED[wheel]) power = -power;
  const float powerAmount = fabsf(power);  // power without the +/- sign
  const MotorPins &pins = motorPins[wheel];

  // ---- Stop ----
  if (powerAmount < 0.001f) {
    if (BRAKE_WHEN_STOPPED) {
      digitalWrite(pins.dirPin1, HIGH);
      digitalWrite(pins.dirPin2, HIGH);
      ledcWrite(pins.pwmPin, PWM_MAX_VALUE);
    } else {
      digitalWrite(pins.dirPin1, LOW);
      digitalWrite(pins.dirPin2, LOW);
      ledcWrite(pins.pwmPin, 0);
    }
    return;
  }

  // ---- Drive ----
  // Map 0..1 onto MIN_MOTOR_POWER..1 so even small commands are strong enough to move.
  const float pwmFraction = MIN_MOTOR_POWER + powerAmount * (1.0f - MIN_MOTOR_POWER);
  const bool forward = power > 0.0f;
  digitalWrite(pins.dirPin1, forward ? HIGH : LOW);
  digitalWrite(pins.dirPin2, forward ? LOW : HIGH);
  ledcWrite(pins.pwmPin, (uint32_t)(pwmFraction * PWM_MAX_VALUE));
}

// =====================================================================================
//                                  SPEED CONTROL (PID)
// =====================================================================================

// PID gains. The PID output is motor power (-1..1) and its input error is in rad/s.
// They can be changed while running with the "P" serial command.
struct PidGains {
  float kp = 0.04f;  // Proportional: push harder the bigger the speed error is now
  float ki = 0.25f;  // Integral:     push harder the longer the error has lasted
  float kd = 0.0f;   // Derivative:   resist sudden speed changes (0 = disabled)
};
PidGains pidGains;

// Everything we know about one wheel.
struct WheelState {
  float targetSpeed      = 0.0f;  // speed the computer wants              (rad/s)
  float position         = 0.0f;  // total angle turned since start/reset  (rad)
  float speed            = 0.0f;  // measured speed, smoothed              (rad/s)
  float integralTerm     = 0.0f;  // the "I" part of the PID, in motor-power units
  int32_t lastEncoderCount = 0;   // encoder count at the previous control step
};
WheelState wheels[NUM_WHEELS];

/// Reads the encoder and updates the wheel's position and smoothed speed.
/// dt = seconds since the previous control step.
void measureWheel(int wheel, float dt) {
  WheelState &w = wheels[wheel];

  const int32_t encoderCount   = readEncoderCount(encoders[wheel]);
  const int32_t countsThisStep = encoderCount - w.lastEncoderCount;
  w.lastEncoderCount = encoderCount;

  // speed = angle turned / time taken, then smoothed to reduce noise
  const float rawSpeed = countsThisStep * RADIANS_PER_ENCODER_COUNT / dt;
  w.speed += VELOCITY_SMOOTHING * (rawSpeed - w.speed);
  w.position = encoderCount * RADIANS_PER_ENCODER_COUNT;
}

/// Calculates the motor power (-1..1) needed to reach the wheel's target speed.
/// previousSpeed = smoothed speed from the previous step (used by the D term).
float computeMotorPower(WheelState &w, float previousSpeed, float dt) {
  // Wheel should stand still: no power (brake/coast) and clear the integral
  // so it does not slowly build up while stopped.
  if (w.targetSpeed == 0.0f) {
    w.integralTerm = 0.0f;
    return 0.0f;
  }

  const float speedError = w.targetSpeed - w.speed;  // positive = too slow

  const float feedforward = USE_FEEDFORWARD ? (w.targetSpeed / MAX_WHEEL_SPEED_RAD_S) : 0.0f;
  const float proportional = pidGains.kp * speedError;
  // D term looks at the change of the measured speed (not the error), so a sudden
  // new target does not cause a power spike.
  const float derivative = -pidGains.kd * (w.speed - previousSpeed) / dt;
  // New integral value, only kept if the anti-windup check below allows it.
  const float newIntegral = w.integralTerm + pidGains.ki * speedError * dt;

  const float requestedPower = feedforward + proportional + newIntegral + derivative;
  const float power = constrain(requestedPower, -1.0f, 1.0f);

  // Anti-windup: if the motor is already at full power, do not keep growing the integral
  // (it would overshoot badly later). Only update it when we are not at the limit, or
  // when the error is pulling the power back away from the limit.
  const bool atPowerLimit = (requestedPower != power);
  const bool errorReducesPower = (requestedPower >  1.0f && speedError < 0.0f) ||
                                 (requestedPower < -1.0f && speedError > 0.0f);
  if (!atPowerLimit || errorReducesPower) {
    w.integralTerm = constrain(newIntegral, -1.0f, 1.0f);
  }

  return power;
}

/// One step of the speed controller for both wheels. dt = seconds since the last step.
void runSpeedControl(float dt) {
  for (int wheel = 0; wheel < NUM_WHEELS; ++wheel) {
    const float previousSpeed = wheels[wheel].speed;
    measureWheel(wheel, dt);
    const float power = computeMotorPower(wheels[wheel], previousSpeed, dt);
    setMotorPower(wheel, power);
  }
}

/// Sets both wheel positions back to zero (used by the "R" command).
void resetEncoders() {
  for (int wheel = 0; wheel < NUM_WHEELS; ++wheel) {
    pcnt_unit_clear_count(encoders[wheel].hardwareCounter);

    // Clear everything about the wheel except the speed the computer asked for.
    const float keepTargetSpeed = wheels[wheel].targetSpeed;
    wheels[wheel] = WheelState{};
    wheels[wheel].targetSpeed = keepTargetSpeed;
  }
}

// =====================================================================================
//                              SERIAL COMMUNICATION
// =====================================================================================

// Times (from millis()/micros()) of the last time each thing happened.
uint32_t lastControlTimeUs = 0;  // last PID step                 (microseconds)
uint32_t lastPublishTimeMs = 0;  // last "S" line sent            (milliseconds)
uint32_t lastCommandTimeMs = 0;  // last valid "V" command received (milliseconds)

// Incoming characters are collected here until a full line has arrived.
char   receivedLine[96];
size_t receivedLength = 0;
bool   lineTooLong = false;  // true = line didn't fit, ignore the rest of it

/// "V <left> <right>" : set the wanted wheel speeds (rad/s).
void handleSpeedCommand(const char *arguments) {
  float leftSpeed, rightSpeed;
  if (sscanf(arguments, "%f %f", &leftSpeed, &rightSpeed) == 2 &&
      isfinite(leftSpeed) && isfinite(rightSpeed)) {
    wheels[LEFT].targetSpeed  = constrain(leftSpeed,  -MAX_WHEEL_SPEED_RAD_S, MAX_WHEEL_SPEED_RAD_S);
    wheels[RIGHT].targetSpeed = constrain(rightSpeed, -MAX_WHEEL_SPEED_RAD_S, MAX_WHEEL_SPEED_RAD_S);
    lastCommandTimeMs = millis();
  }
}

/// "P <kp> <ki> <kd>" : change the PID gains.
void handlePidCommand(const char *arguments) {
  float kp, ki, kd;
  if (sscanf(arguments, "%f %f %f", &kp, &ki, &kd) == 3 &&
      isfinite(kp) && isfinite(ki) && isfinite(kd)) {
    pidGains = {kp, ki, kd};
    wheels[LEFT].integralTerm = wheels[RIGHT].integralTerm = 0.0f;
    Serial.printf("A PID %.5f %.5f %.5f\n", pidGains.kp, pidGains.ki, pidGains.kd);
  }
}

/// Runs one complete command line. The first letter says which command it is.
void handleCommand(char *line) {
  const char commandLetter = line[0];
  const char *arguments = line + 1;

  switch (commandLetter) {
    case 'V':
      handleSpeedCommand(arguments);
      break;
    case 'P':
      handlePidCommand(arguments);
      break;
    case 'R':
      resetEncoders();
      Serial.println("A RESET");
      break;
    default:
      break;  // unknown command: ignore
  }
}

/// Reads all waiting characters. When a line ending ('\n' or '\r') arrives,
/// the collected line is handled as a command.
void readSerialCommands() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      // End of line: run it (unless it was empty or too long), then start a new line.
      if (receivedLength > 0 && !lineTooLong) {
        receivedLine[receivedLength] = '\0';
        handleCommand(receivedLine);
      }
      receivedLength = 0;
      lineTooLong = false;
    } else if (lineTooLong) {
      // still inside a line that was too long: throw these characters away
    } else if (receivedLength < sizeof(receivedLine) - 1) {
      receivedLine[receivedLength++] = c;  // room left (keep 1 byte for the '\0')
    } else {
      lineTooLong = true;
    }
  }
}

/// Sends "S <left_pos> <right_pos> <left_speed> <right_speed>" to the computer.
void publishWheelState() {
  char message[96];
  const int length = snprintf(message, sizeof(message), "S %.4f %.4f %.4f %.4f\n",
                              wheels[LEFT].position, wheels[RIGHT].position,
                              wheels[LEFT].speed,    wheels[RIGHT].speed);
  if (length > 0) Serial.write((const uint8_t *)message, (size_t)length);
}

// =====================================================================================
//                                  ARDUINO SETUP / LOOP
// =====================================================================================

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  for (int wheel = 0; wheel < NUM_WHEELS; ++wheel) {
    setupMotor(wheel);
    setMotorPower(wheel, 0.0f);  // start stopped
  }

  for (Encoder &encoder : encoders) {
    setupEncoder(encoder);
  }

  lastControlTimeUs = micros();
  lastPublishTimeMs = millis();
  lastCommandTimeMs = millis();
  Serial.println("A READY");
}

void loop() {
  // 1. Handle any commands from the computer.
  readSerialCommands();

  // 2. Every CONTROL_LOOP_PERIOD_US: update the speed controller.
  const uint32_t nowUs = micros();
  if (nowUs - lastControlTimeUs >= CONTROL_LOOP_PERIOD_US) {
    const float secondsSinceLastStep = (nowUs - lastControlTimeUs) * 1e-6f;
    lastControlTimeUs = nowUs;

    // Safety: if the computer stopped sending speed commands, stop the robot.
    if (millis() - lastCommandTimeMs > COMMAND_TIMEOUT_MS) {
      wheels[LEFT].targetSpeed  = 0.0f;
      wheels[RIGHT].targetSpeed = 0.0f;
    }
    runSpeedControl(secondsSinceLastStep);
  }

  // 3. Every STATE_PUBLISH_PERIOD_MS: send wheel position and speed to the computer.
  const uint32_t nowMs = millis();
  if (nowMs - lastPublishTimeMs >= STATE_PUBLISH_PERIOD_MS) {
    lastPublishTimeMs = nowMs;
    publishWheelState();
  }
}
