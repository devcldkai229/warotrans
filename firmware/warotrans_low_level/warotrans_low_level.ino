#include <Arduino.h>
#include <driver/gpio.h>
#include <esp_arduino_version.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// MDD10A wiring:
//   Left side:  PWM1 -> GPIO 4, DIR1 -> GPIO 5
//   Right side: PWM2 -> GPIO 6, DIR2 -> GPIO 7
constexpr uint8_t left_pwm_pin = 4;
constexpr uint8_t left_dir_pin = 5;
constexpr uint8_t right_pwm_pin = 6;
constexpr uint8_t right_dir_pin = 7;

// Encoder wiring:
//   Left:  C2 -> GPIO 8,  C1 -> GPIO 9
//   Right: C2 -> GPIO 10, C1 -> GPIO 11
constexpr uint8_t left_encoder_c2_pin = 8;
constexpr uint8_t left_encoder_c1_pin = 9;
constexpr uint8_t right_encoder_c2_pin = 10;
constexpr uint8_t right_encoder_c1_pin = 11;

// COMMISSIONING SCALE — NOT a measured vehicle speed.
// Command setpoint only; real velocity must come from encoder odometry
// and Phase 7B physical testing.
constexpr float commissioning_full_scale_linear = 0.32F;  // m/s at 100% duty
constexpr float wheel_separation = 0.4535F;  // effective, operator 2026-09-07
constexpr float commissioning_full_scale_angular =
    2.0F * commissioning_full_scale_linear / wheel_separation;  // ~1.411 rad/s

// Motor polarity — commissioning configuration, flip if a side runs backwards.
// Commissioning 2026-09-04: FORWARD(+linear) turned left and LEFT(+angular)
 // drove straight — left channel polarity inverted relative to mixer sign.
 // After flashing this true, restore standard teleop mapping in web_teleop_node
 // (FORWARD=±linear.x, LEFT=±angular.z) and remove the temporary remap.
constexpr bool LEFT_MOTOR_INVERTED = true;
constexpr bool RIGHT_MOTOR_INVERTED = false;

// Hard PWM safety ceiling for commissioning.
constexpr uint8_t MOTOR_PWM_ABSOLUTE_MAX_PERCENT = 40;
constexpr uint8_t MOTOR_PWM_LIMIT_PERCENT = 40;
static_assert(MOTOR_PWM_LIMIT_PERCENT <= MOTOR_PWM_ABSOLUTE_MAX_PERCENT,
              "PWM limit exceeds commissioning ceiling");

// Keep the tested encoder sign behavior from encoder_test.
constexpr int8_t left_encoder_sign = -1;
constexpr int8_t right_encoder_sign = 1;

constexpr uint32_t serial_baud = 921600;
constexpr uint32_t report_period_ms = 20;
constexpr uint32_t command_watchdog_ms = 300;

// TUNING START VALUE. Verify against the actual MDD10A and motor setup.
constexpr uint32_t pwm_frequency_hz = 5000;
constexpr uint8_t pwm_resolution_bits = 8;
constexpr uint32_t max_pwm_duty =
    (static_cast<uint32_t>(1) << pwm_resolution_bits) - 1U;
constexpr uint32_t motor_pwm_limit_duty =
    (max_pwm_duty * MOTOR_PWM_LIMIT_PERCENT) / 100U;  // 40% of 255 = 102

// ---------------------------------------------------------------------------
// Wheel-velocity closed-loop scaffold (Class B actuator mismatch)
// WHY: Nav2 assumes SI body twist is approximately realized; open-loop PWM
//      may leave /cmd_vel.angular.z unmet while chassis keeps going forward.
// DEFAULT OFF — keep current open-loop behavior until bench PID calib.
// RISK: non-zero gains without measurement can overshoot / fight motors.
// ROLLBACK: leave USE_WHEEL_VELOCITY_CLOSED_LOOP false, or reflash prior build.
// Calib constants mirror ros2_ws/.../hardware.yaml (operator 2026-09-07).
// ---------------------------------------------------------------------------
constexpr bool USE_WHEEL_VELOCITY_CLOSED_LOOP = false;
// PROVISIONAL — require physical step-response / in-place 90° calib. Do NOT invent.
constexpr float WHEEL_VEL_KP = 0.0F;
constexpr float WHEEL_VEL_KI = 0.0F;
constexpr float WHEEL_VEL_KD = 0.0F;
constexpr float odom_ticks_per_rev_left = 2478.1F;
constexpr float odom_ticks_per_rev_right = 2478.1F;
constexpr float odom_wheel_radius_left = 0.03296F;
constexpr float odom_wheel_radius_right = 0.03263F;
constexpr uint32_t wheel_vel_control_period_ms = 20;

constexpr size_t command_buffer_size = 64;

volatile int32_t left_ticks = 0;
volatile int32_t right_ticks = 0;
volatile uint8_t left_state = 0;
volatile uint8_t right_state = 0;

portMUX_TYPE encoder_mux = portMUX_INITIALIZER_UNLOCKED;

// Index: previous_state << 2 | current_state.
// State bits are ordered as C2:C1. Valid transitions are decoded as x4.
DRAM_ATTR static const int8_t quadrature_delta[16] = {
    0, 1, -1, 0,
    -1, 0, 0, 1,
    1, 0, 0, -1,
    0, -1, 1, 0,
};

char command_buffer[command_buffer_size];
size_t command_length = 0;
bool discard_command = false;

bool left_pwm_attached = false;
bool right_pwm_attached = false;
bool pwm_ready = false;
bool valid_velocity_command = false;
uint32_t last_valid_command_ms = 0;
uint32_t next_report_ms = 0;
uint32_t next_wheel_vel_control_ms = 0;

// Closed-loop targets / PID state (used only when USE_WHEEL_VELOCITY_CLOSED_LOOP)
float target_left_mps = 0.0F;
float target_right_mps = 0.0F;
bool have_wheel_targets = false;
int32_t speed_sample_left_ticks = 0;
int32_t speed_sample_right_ticks = 0;
uint32_t speed_sample_ms = 0;
float pid_integral_left = 0.0F;
float pid_integral_right = 0.0F;
float pid_last_err_left = 0.0F;
float pid_last_err_right = 0.0F;

IRAM_ATTR uint8_t read_encoder_state(uint8_t c2_pin, uint8_t c1_pin) {
  const uint8_t c2 = static_cast<uint8_t>(
      gpio_get_level(static_cast<gpio_num_t>(c2_pin)));
  const uint8_t c1 = static_cast<uint8_t>(
      gpio_get_level(static_cast<gpio_num_t>(c1_pin)));
  return static_cast<uint8_t>((c2 << 1) | c1);
}

IRAM_ATTR void update_encoder(volatile int32_t& ticks,
                              volatile uint8_t& previous_state,
                              uint8_t c2_pin,
                              uint8_t c1_pin) {
  portENTER_CRITICAL_ISR(&encoder_mux);

  const uint8_t current_state = read_encoder_state(c2_pin, c1_pin);
  const uint8_t transition =
      static_cast<uint8_t>((previous_state << 2) | current_state);
  ticks += quadrature_delta[transition];
  previous_state = current_state;

  portEXIT_CRITICAL_ISR(&encoder_mux);
}

void IRAM_ATTR on_left_c2_change() {
  update_encoder(left_ticks, left_state, left_encoder_c2_pin,
                 left_encoder_c1_pin);
}

void IRAM_ATTR on_left_c1_change() {
  update_encoder(left_ticks, left_state, left_encoder_c2_pin,
                 left_encoder_c1_pin);
}

void IRAM_ATTR on_right_c2_change() {
  update_encoder(right_ticks, right_state, right_encoder_c2_pin,
                 right_encoder_c1_pin);
}

void IRAM_ATTR on_right_c1_change() {
  update_encoder(right_ticks, right_state, right_encoder_c2_pin,
                 right_encoder_c1_pin);
}

void reset_ticks() {
  portENTER_CRITICAL(&encoder_mux);
  left_ticks = 0;
  right_ticks = 0;
  portEXIT_CRITICAL(&encoder_mux);
}

void copy_ticks(int32_t& left, int32_t& right) {
  portENTER_CRITICAL(&encoder_mux);
  left = left_ticks;
  right = right_ticks;
  portEXIT_CRITICAL(&encoder_mux);
}

bool is_whitespace(char value) {
  return value == ' ' || value == '\t';
}

void skip_whitespace(const char*& cursor) {
  while (is_whitespace(*cursor)) {
    ++cursor;
  }
}

bool parse_float_token(const char*& cursor, float& value) {
  skip_whitespace(cursor);
  if (*cursor == '\0') {
    return false;
  }

  char* end = nullptr;
  value = strtof(cursor, &end);
  if (end == cursor || !isfinite(value)) {
    return false;
  }

  cursor = end;
  return true;
}

bool parse_velocity_command(const char* line,
                            float& linear_mps,
                            float& angular_rps) {
  const char* cursor = line;
  skip_whitespace(cursor);
  if (*cursor != 'V') {
    return false;
  }
  ++cursor;

  if (!is_whitespace(*cursor)) {
    return false;
  }

  if (!parse_float_token(cursor, linear_mps) ||
      !parse_float_token(cursor, angular_rps)) {
    return false;
  }

  skip_whitespace(cursor);
  return *cursor == '\0';
}

bool is_reset_command(const char* line) {
  const char* cursor = line;
  skip_whitespace(cursor);
  if (*cursor != 'R') {
    return false;
  }
  ++cursor;
  skip_whitespace(cursor);
  return *cursor == '\0';
}

bool motion_config_ready() {
  return isfinite(wheel_separation) && wheel_separation > 0.0F &&
         isfinite(commissioning_full_scale_linear) &&
         commissioning_full_scale_linear > 0.0F &&
         isfinite(commissioning_full_scale_angular) &&
         commissioning_full_scale_angular > 0.0F;
}

uint32_t duty_from_wheel_speed(float wheel_speed) {
  if (!isfinite(wheel_speed) || commissioning_full_scale_linear <= 0.0F) {
    return 0;
  }

  float ratio = fabsf(wheel_speed) / commissioning_full_scale_linear;
  if (!isfinite(ratio) || ratio <= 0.0F) {
    return 0;
  }
  if (ratio > 1.0F) {
    ratio = 1.0F;
  }

  const float duty = ratio * static_cast<float>(max_pwm_duty);
  if (duty >= static_cast<float>(motor_pwm_limit_duty)) {
    return motor_pwm_limit_duty;
  }
  return static_cast<uint32_t>(lroundf(duty));
}

void write_pwm(uint8_t pwm_pin, uint32_t duty) {
  if (duty > motor_pwm_limit_duty) {
    duty = motor_pwm_limit_duty;
  }

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pwm_pin, duty);
#else
  const uint8_t channel = (pwm_pin == left_pwm_pin) ? 0 : 1;
  ledcWrite(channel, duty);
#endif
}

void stop_motors() {
  if (left_pwm_attached) {
    write_pwm(left_pwm_pin, 0);
  } else {
    digitalWrite(left_pwm_pin, LOW);
  }
  if (right_pwm_attached) {
    write_pwm(right_pwm_pin, 0);
  } else {
    digitalWrite(right_pwm_pin, LOW);
  }

  digitalWrite(left_dir_pin, LOW);
  digitalWrite(right_dir_pin, LOW);

  have_wheel_targets = false;
  target_left_mps = 0.0F;
  target_right_mps = 0.0F;
  pid_integral_left = 0.0F;
  pid_integral_right = 0.0F;
  pid_last_err_left = 0.0F;
  pid_last_err_right = 0.0F;
}

void write_motor_velocity(uint8_t pwm_pin,
                          uint8_t dir_pin,
                          float wheel_speed,
                          bool inverted) {
  const bool forward = wheel_speed >= 0.0F;
  digitalWrite(dir_pin, (forward != inverted) ? HIGH : LOW);
  write_pwm(pwm_pin, duty_from_wheel_speed(wheel_speed));
}

void write_motor_duty(uint8_t pwm_pin,
                      uint8_t dir_pin,
                      float wheel_speed_sign,
                      uint32_t duty,
                      bool inverted) {
  const bool forward = wheel_speed_sign >= 0.0F;
  digitalWrite(dir_pin, (forward != inverted) ? HIGH : LOW);
  write_pwm(pwm_pin, duty);
}

void apply_wheel_velocities(float left_speed, float right_speed) {
  if (!pwm_ready) {
    stop_motors();
    return;
  }

  write_motor_velocity(left_pwm_pin, left_dir_pin, left_speed,
                       LEFT_MOTOR_INVERTED);
  write_motor_velocity(right_pwm_pin, right_dir_pin, right_speed,
                       RIGHT_MOTOR_INVERTED);
}

float ticks_to_mps(int32_t delta_ticks,
                   float ticks_per_rev,
                   float wheel_radius,
                   float dt_s) {
  if (dt_s <= 0.0F || ticks_per_rev <= 0.0F || wheel_radius <= 0.0F) {
    return 0.0F;
  }
  const float meters =
      (static_cast<float>(delta_ticks) / ticks_per_rev) *
      (2.0F * 3.14159265358979323846F * wheel_radius);
  return meters / dt_s;
}

uint32_t clamp_duty_u32(float duty_f) {
  if (!isfinite(duty_f) || duty_f <= 0.0F) {
    return 0;
  }
  if (duty_f >= static_cast<float>(motor_pwm_limit_duty)) {
    return motor_pwm_limit_duty;
  }
  return static_cast<uint32_t>(lroundf(duty_f));
}

void reset_wheel_speed_sampler(uint32_t now) {
  int32_t left_snapshot = 0;
  int32_t right_snapshot = 0;
  copy_ticks(left_snapshot, right_snapshot);
  // Same sign convention as telemetry "O" lines (encoder_sign applied).
  speed_sample_left_ticks = left_snapshot * left_encoder_sign;
  speed_sample_right_ticks = right_snapshot * right_encoder_sign;
  speed_sample_ms = now;
}

void update_wheel_velocity_closed_loop(uint32_t now) {
  if (!USE_WHEEL_VELOCITY_CLOSED_LOOP || !have_wheel_targets || !pwm_ready) {
    return;
  }
  if (static_cast<int32_t>(now - next_wheel_vel_control_ms) < 0) {
    return;
  }
  next_wheel_vel_control_ms = now + wheel_vel_control_period_ms;

  int32_t left_snapshot = 0;
  int32_t right_snapshot = 0;
  copy_ticks(left_snapshot, right_snapshot);
  const int32_t signed_left = left_snapshot * left_encoder_sign;
  const int32_t signed_right = right_snapshot * right_encoder_sign;
  const float dt_s =
      static_cast<float>(now - speed_sample_ms) * 0.001F;
  const float meas_left = ticks_to_mps(
      signed_left - speed_sample_left_ticks, odom_ticks_per_rev_left,
      odom_wheel_radius_left, dt_s);
  const float meas_right = ticks_to_mps(
      signed_right - speed_sample_right_ticks, odom_ticks_per_rev_right,
      odom_wheel_radius_right, dt_s);
  speed_sample_left_ticks = signed_left;
  speed_sample_right_ticks = signed_right;
  speed_sample_ms = now;

  const float err_l = target_left_mps - meas_left;
  const float err_r = target_right_mps - meas_right;
  pid_integral_left += err_l * dt_s;
  pid_integral_right += err_r * dt_s;
  const float d_l =
      (dt_s > 0.0F) ? (err_l - pid_last_err_left) / dt_s : 0.0F;
  const float d_r =
      (dt_s > 0.0F) ? (err_r - pid_last_err_right) / dt_s : 0.0F;
  pid_last_err_left = err_l;
  pid_last_err_right = err_r;

  // Feedforward (open-loop map) + PID. With K*=0 this equals open-loop duty.
  const float ff_l = static_cast<float>(duty_from_wheel_speed(target_left_mps));
  const float ff_r =
      static_cast<float>(duty_from_wheel_speed(target_right_mps));
  const float corr_l =
      WHEEL_VEL_KP * err_l + WHEEL_VEL_KI * pid_integral_left +
      WHEEL_VEL_KD * d_l;
  const float corr_r =
      WHEEL_VEL_KP * err_r + WHEEL_VEL_KI * pid_integral_right +
      WHEEL_VEL_KD * d_r;

  const uint32_t duty_l = clamp_duty_u32(ff_l + corr_l);
  const uint32_t duty_r = clamp_duty_u32(ff_r + corr_r);
  write_motor_duty(left_pwm_pin, left_dir_pin, target_left_mps, duty_l,
                   LEFT_MOTOR_INVERTED);
  write_motor_duty(right_pwm_pin, right_dir_pin, target_right_mps, duty_r,
                   RIGHT_MOTOR_INVERTED);
}

void handle_velocity_command(float linear_mps, float angular_rps) {
  if (!isfinite(linear_mps) || !isfinite(angular_rps)) {
    stop_motors();
    return;
  }

  if (linear_mps == 0.0F && angular_rps == 0.0F) {
    stop_motors();
    valid_velocity_command = true;
    last_valid_command_ms = millis();
    return;
  }

  if (!motion_config_ready()) {
    stop_motors();
    return;
  }

  linear_mps = constrain(linear_mps, -commissioning_full_scale_linear,
                         commissioning_full_scale_linear);
  angular_rps = constrain(angular_rps, -commissioning_full_scale_angular,
                          commissioning_full_scale_angular);

  // Skid-steer: left = v - w*W/2, right = v + w*W/2 (W effective 0.4535).
  float left_speed =
      linear_mps - angular_rps * wheel_separation / 2.0F;
  float right_speed =
      linear_mps + angular_rps * wheel_separation / 2.0F;

  const float peak = fmaxf(fabsf(left_speed), fabsf(right_speed));
  if (peak > commissioning_full_scale_linear) {
    const float scale = commissioning_full_scale_linear / peak;
    left_speed *= scale;
    right_speed *= scale;
  }

  if (!isfinite(left_speed) || !isfinite(right_speed)) {
    stop_motors();
    return;
  }

  target_left_mps = left_speed;
  target_right_mps = right_speed;
  have_wheel_targets = true;
  valid_velocity_command = true;
  last_valid_command_ms = millis();

  if (USE_WHEEL_VELOCITY_CLOSED_LOOP) {
    reset_wheel_speed_sampler(last_valid_command_ms);
    next_wheel_vel_control_ms = last_valid_command_ms;
    update_wheel_velocity_closed_loop(last_valid_command_ms);
  } else {
    apply_wheel_velocities(left_speed, right_speed);
  }
}

void handle_command(const char* line) {
  if (is_reset_command(line)) {
    reset_ticks();
    return;
  }

  float linear_mps = 0.0F;
  float angular_rps = 0.0F;
  if (parse_velocity_command(line, linear_mps, angular_rps)) {
    handle_velocity_command(linear_mps, angular_rps);
  }
}

void process_serial_commands() {
  while (Serial.available() > 0) {
    const char incoming = static_cast<char>(Serial.read());

    if (incoming == '\r') {
      continue;
    }

    if (incoming == '\n') {
      if (!discard_command) {
        command_buffer[command_length] = '\0';
        handle_command(command_buffer);
      }
      command_length = 0;
      discard_command = false;
      continue;
    }

    if (discard_command) {
      continue;
    }

    if (command_length < command_buffer_size - 1) {
      command_buffer[command_length++] = incoming;
    } else {
      command_length = 0;
      discard_command = true;
    }
  }
}

bool configure_pwm() {
  bool left_attached = false;
  bool right_attached = false;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  left_attached =
      ledcAttach(left_pwm_pin, pwm_frequency_hz, pwm_resolution_bits);
  right_attached =
      ledcAttach(right_pwm_pin, pwm_frequency_hz, pwm_resolution_bits);
#else
  left_attached =
      ledcSetup(0, pwm_frequency_hz, pwm_resolution_bits) != 0;
  right_attached =
      ledcSetup(1, pwm_frequency_hz, pwm_resolution_bits) != 0;
  if (left_attached) {
    ledcAttachPin(left_pwm_pin, 0);
  }
  if (right_attached) {
    ledcAttachPin(right_pwm_pin, 1);
  }
#endif

  left_pwm_attached = left_attached;
  right_pwm_attached = right_attached;
  if (left_pwm_attached) {
    write_pwm(left_pwm_pin, 0);
  }
  if (right_pwm_attached) {
    write_pwm(right_pwm_pin, 0);
  }
  return left_pwm_attached && right_pwm_attached;
}

void setup_encoder_inputs() {
  pinMode(left_encoder_c2_pin, INPUT);
  pinMode(left_encoder_c1_pin, INPUT);
  pinMode(right_encoder_c2_pin, INPUT);
  pinMode(right_encoder_c1_pin, INPUT);

  portENTER_CRITICAL(&encoder_mux);
  left_state =
      read_encoder_state(left_encoder_c2_pin, left_encoder_c1_pin);
  right_state =
      read_encoder_state(right_encoder_c2_pin, right_encoder_c1_pin);
  left_ticks = 0;
  right_ticks = 0;
  portEXIT_CRITICAL(&encoder_mux);

  attachInterrupt(digitalPinToInterrupt(left_encoder_c2_pin),
                  on_left_c2_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_encoder_c1_pin),
                  on_left_c1_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_encoder_c2_pin),
                  on_right_c2_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_encoder_c1_pin),
                  on_right_c1_change, CHANGE);
}

void report_telemetry(uint32_t now) {
  if (static_cast<int32_t>(now - next_report_ms) < 0) {
    return;
  }

  next_report_ms += report_period_ms;

  int32_t left_snapshot = 0;
  int32_t right_snapshot = 0;
  copy_ticks(left_snapshot, right_snapshot);

  const long output_left =
      static_cast<long>(left_snapshot * left_encoder_sign);
  const long output_right =
      static_cast<long>(right_snapshot * right_encoder_sign);
  Serial.printf("O %ld %ld %lu\n", output_left, output_right,
                static_cast<unsigned long>(now));
}

void setup() {
  // Drive PWM low before configuring the rest of the hardware.
  pinMode(left_pwm_pin, OUTPUT);
  digitalWrite(left_pwm_pin, LOW);
  pinMode(right_pwm_pin, OUTPUT);
  digitalWrite(right_pwm_pin, LOW);
  pinMode(left_dir_pin, OUTPUT);
  digitalWrite(left_dir_pin, LOW);
  pinMode(right_dir_pin, OUTPUT);
  digitalWrite(right_dir_pin, LOW);

  setup_encoder_inputs();
  pwm_ready = configure_pwm();
  stop_motors();

  Serial.begin(serial_baud);
  next_report_ms = millis() + report_period_ms;
}

void loop() {
  const uint32_t now = millis();
  if (!valid_velocity_command ||
      now - last_valid_command_ms >= command_watchdog_ms) {
    stop_motors();
  } else if (USE_WHEEL_VELOCITY_CLOSED_LOOP) {
    update_wheel_velocity_closed_loop(now);
  }

  process_serial_commands();
  report_telemetry(now);
}
