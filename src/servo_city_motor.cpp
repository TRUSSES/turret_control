#include "servo_city_motor.h"
#include <algorithm>
#include <iostream>

ServoCityMotor* ServoCityMotor::instance = nullptr;

ServoCityMotor::ServoCityMotor(int pwm_pin, int dir_pin, int enc_a, int enc_b)
    : pwm_pin_(pwm_pin),
      dir_pin_(dir_pin),
      enc_a_(enc_a),
      enc_b_(enc_b) {
  // Initialize pigpio
  if (gpioInitialise() < 0) {
    std::cerr << "pigpio initialization failed" << std::endl;
    exit(1);
  }

  // Configure PWM pin and direction pin
  gpioSetMode(pwm_pin_, PI_OUTPUT);
  gpioSetMode(dir_pin_, PI_OUTPUT);
  // Set PWM frequency to 10 kHz (adjustable; around 20 kHz is recommended for sign-magnitude)
  gpioSetPWMfrequency(pwm_pin_, 10000);
  gpioWrite(dir_pin_, 0);

  // Configure encoder pins
  gpioSetMode(enc_a_, PI_INPUT);
  gpioSetMode(enc_b_, PI_INPUT);
  gpioSetPullUpDown(enc_a_, PI_PUD_UP);
  gpioSetPullUpDown(enc_b_, PI_PUD_UP);
  gpioGlitchFilter(enc_a_, 1000);
  gpioGlitchFilter(enc_b_, 1000);

  // Register encoder ISR for both encoder pins
  gpioSetISRFunc(enc_a_, EITHER_EDGE, 0, encoderISR);
  gpioSetISRFunc(enc_b_, EITHER_EDGE, 0, encoderISR);

  // Set static instance for ISR callbacks.
  instance = this;

  last_update_ = std::chrono::steady_clock::now();
  last_encoder_time_ = std::chrono::steady_clock::now();

  // Debug messages to verify initialization.
  std::cout << "ServoCityMotor initialized:" << std::endl;
  std::cout << "  PWM Pin: " << pwm_pin_ << std::endl;
  std::cout << "  Dir Pin: " << dir_pin_ << std::endl;
  std::cout << "  Encoder A: " << enc_a_ << std::endl;
  std::cout << "  Encoder B: " << enc_b_ << std::endl;
}

ServoCityMotor::ServoCityMotor(const YAML::Node &node)
    : ServoCityMotor(node["servo_pwm_pin"].as<int>(),
                     node["servo_dir_pin"].as<int>(),
                     node["servo_enc_a"].as<int>(),
                     node["servo_enc_b"].as<int>()) {
  // Debug print loaded configuration.
  std::cout << "ServoCityMotor constructed from YAML config:" << std::endl;
  std::cout << "  servo_pwm_pin: " << node["servo_pwm_pin"].as<int>() << std::endl;
  std::cout << "  servo_dir_pin: " << node["servo_dir_pin"].as<int>() << std::endl;
  std::cout << "  servo_enc_a: " << node["servo_enc_a"].as<int>() << std::endl;
  std::cout << "  servo_enc_b: " << node["servo_enc_b"].as<int>() << std::endl;
}

ServoCityMotor::~ServoCityMotor() {
  gpioTerminate();
}

void ServoCityMotor::setTargetVelocity(double target_rad_per_sec) {
  std::cout << "Setting target velocity: " << target_rad_per_sec << " rad/s" << std::endl;
  target_velocity_ = target_rad_per_sec;
}

double ServoCityMotor::getCurrentVelocity() const {
  return current_velocity_.load();
}

void ServoCityMotor::update() {
  auto now = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(now - last_update_).count();
  last_update_ = now;

  double current = current_velocity_.load();
  double error = target_velocity_ - current;

  integral_ += error * dt;
  integral_ = std::clamp(integral_, -1.0, 1.0);
  double derivative = (error - prev_error_) / dt;
  prev_error_ = error;

  double output = Kp_ * error + Ki_ * integral_ + Kd_ * derivative;
  output = std::clamp(output, -1.0, 1.0);

  // Filter the output for stability.
  output_filter_ = (1.0 - filter_gain_) * output_filter_ + filter_gain_ * output;
  //std::cout << "Velocity output filter value: " << output_filter_ << std::endl;

  // Sign-magnitude PWM:
  // If the filtered output is below threshold, brake by setting PWM to 0.
  const double threshold = 0.05;
  int duty_cycle = 0;
  if (fabs(output_filter_) > threshold) {
    // Calculate the duty cycle based on the absolute output value.
    duty_cycle = static_cast<int>(fabs(output_filter_) * 255.0);
    // Enforce a minimum duty cycle (around 5%).
    if (duty_cycle < 13) {
      duty_cycle = 13;
    }
    // Set the direction pin based on the sign.
    gpioWrite(dir_pin_, output_filter_ > 0 ? 1 : 0);
  } else {
    duty_cycle = 0;
  }

  // Generate the PWM signal with the calculated duty cycle.
  gpioPWM(pwm_pin_, duty_cycle);

  auto now_enc = std::chrono::steady_clock::now();
  double dt_enc = std::chrono::duration<double>(now_enc - last_encoder_time_).count();
  if(dt_enc > 0.01) {
    int counts = encoder_count.exchange(0);
    double delta_rad = countsToRadians(counts);
    double raw_velocity = delta_rad / dt_enc;

    velocity_filter_ = (1.0 - velocity_filter_gain_) * velocity_filter_ +
                       velocity_filter_gain_ * raw_velocity;
    current_velocity_ = velocity_filter_;
    last_encoder_time_ = now_enc;
  }
}

void ServoCityMotor::encoderISR(int gpio, int level, uint32_t tick) {
  if(instance) {
    int a = gpioRead(instance->enc_a_);
    int b = gpioRead(instance->enc_b_);
    if(instance->debug_encoder) {
      instance->raw_a = a;
      instance->raw_b = b;
    }
    instance->updateEncoder(a, b);
  }
}

void ServoCityMotor::updateEncoder(int a, int b) {
  static int last_encoded = 0;
  int encoded = (a << 1) | b;
  int sum = (last_encoded << 2) | encoded;

  const int8_t decoder[16] = {
      0,  -1,  1,  0,
      1,   0,  0, -1,
      -1,  0,  0,  1,
      0,   1, -1,  0
  };

  encoder_count += decoder[sum];
  last_encoded = encoded;
}

double ServoCityMotor::countsToRadians(int counts) const {
  double output_revs = static_cast<double>(counts) / counts_per_rev_;
  return output_revs * 2 * M_PI;
}

void ServoCityMotor::stop() {
  gpioPWM(pwm_pin_, 0);
  integral_ = 0.0;
  prev_error_ = 0.0;
}

void ServoCityMotor::setMotorOutput(int pwm) {
  gpioServo(pwm_pin_, pwm);
}
