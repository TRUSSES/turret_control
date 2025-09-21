#include "servo_city_motor.h"
#include <algorithm>
#include <iostream>

ServoCityMotor* ServoCityMotor::instance_[4] = {nullptr, nullptr, nullptr, nullptr};
int ServoCityMotor::instance_count_ = 0;

void ServoCityMotor::encoderISR(int gpio, int level, uint32_t tick) {
  // Find which motor instance this GPIO belongs to
  for (int i = 0; i < instance_count_; i++) {
    if (instance_[i] && (gpio == instance_[i]->enc_a_ || gpio == instance_[i]->enc_b_)) {
      int a = gpioRead(instance_[i]->enc_a_);
      int b = gpioRead(instance_[i]->enc_b_);
      if (instance_[i]->debug_encoder) {
        instance_[i]->raw_a = a;
        instance_[i]->raw_b = b;
      }
      instance_[i]->updateEncoder(a, b);
      break;
    }
  }
}

ServoCityMotor::ServoCityMotor(int pwm_pin, int dir_pin, int enc_a, int enc_b)
    : pwm_pin_(pwm_pin),
      dir_pin_(dir_pin),
      enc_a_(enc_a),
      enc_b_(enc_b) {
  // Initialize pigpio only if not already initialized
  static bool pigpio_initialized = false;
  if (!pigpio_initialized) {
    if (gpioInitialise() < 0) {
      std::cerr << "pigpio initialization failed" << std::endl;
      exit(1);
    }
    pigpio_initialized = true;
  }

  // Configure PWM pin and direction pin for Pololu 18v25
  gpioSetMode(pwm_pin_, PI_OUTPUT);
  gpioSetMode(dir_pin_, PI_OUTPUT);
  
  // Set initial states
  gpioPWM(pwm_pin_, 0);          // Start with PWM off
  gpioWrite(dir_pin_, 0);        // Start with direction low
  
  // Set PWM frequency to 20 kHz (recommended for Pololu 18v25)
  gpioSetPWMfrequency(pwm_pin_, 20000);
  
  std::cout << "Motor GPIO initialized: PWM=" << pwm_pin_ << " DIR=" << dir_pin_ << std::endl;

  // Configure encoder pins (but disable ISRs for testing)
  gpioSetMode(enc_a_, PI_INPUT);
  gpioSetMode(enc_b_, PI_INPUT);
  gpioSetPullUpDown(enc_a_, PI_PUD_UP);
  gpioSetPullUpDown(enc_b_, PI_PUD_UP);
  gpioGlitchFilter(enc_a_, 1000);
  gpioGlitchFilter(enc_b_, 1000);

  // Enable encoder ISRs for PID control
  gpioSetISRFunc(enc_a_, EITHER_EDGE, 0, encoderISR);
  gpioSetISRFunc(enc_b_, EITHER_EDGE, 0, encoderISR);
  
  std::cout << "Encoder ISRs ENABLED for PID control" << std::endl;

  // Add this instance to the static array
  if (instance_count_ < 4) {
    instance_id_ = instance_count_;
    instance_[instance_count_] = this;
    instance_count_++;
  }

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
    : ServoCityMotor(node["pwm"].as<int>(),
                     node["dir"].as<int>(),
                     node["enca"].as<int>(),
                     node["encb"].as<int>()) {
  
  // Load individual PID gains if provided
  if (node["pid"]) {
    auto pid = node["pid"];
    if (pid["kp"]) Kp_ = pid["kp"].as<double>();
    if (pid["ki"]) Ki_ = pid["ki"].as<double>();
    if (pid["kd"]) Kd_ = pid["kd"].as<double>();
  }
  
  // Debug print loaded configuration.
  std::cout << "ServoCityMotor constructed from YAML config:" << std::endl;
  std::cout << "  PWM: " << node["pwm"].as<int>() << " DIR: " << node["dir"].as<int>() << std::endl;
  std::cout << "  PID gains: Kp=" << Kp_ << " Ki=" << Ki_ << " Kd=" << Kd_ << std::endl;
}

ServoCityMotor::~ServoCityMotor() {
  stop();
  // Remove this instance from the static array
  if (instance_id_ < instance_count_) {
    instance_[instance_id_] = nullptr;
  }
}

void ServoCityMotor::setTargetVelocity(double target_rad_per_sec) {
  //std::cout << "[Motor PWM:" << pwm_pin_ << "] setTargetVelocity: " << target_rad_per_sec << " rad/s" << std::endl;
  target_velocity_ = target_rad_per_sec;
}

double ServoCityMotor::getTargetVelocity() {
  //std::cout << "[Motor PWM:" << pwm_pin_ << "] setTargetVelocity: " << target_rad_per_sec << " rad/s" << std::endl;
  return target_velocity_;
}

double ServoCityMotor::getCurrentVelocity() const {
  return current_velocity_.load();
}

void ServoCityMotor::update() {
  auto now = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(now - last_update_).count();
  last_update_ = now;
  
  if (dt <= 0.0) return;

  // Calculate velocity from encoder
  auto now_enc = std::chrono::steady_clock::now();
  double dt_enc = std::chrono::duration<double>(now_enc - last_encoder_time_).count();
  if(dt_enc > 0.05) { // 50ms minimum for stable reading
    int counts = encoder_count.exchange(0);
    double delta_rad = countsToRadians(counts);
    current_velocity_ = delta_rad / dt_enc;
    last_encoder_time_ = now_enc;
  }

  // Full PID control with all three terms
  double current = current_velocity_.load();
  double error = target_velocity_ - current;
  
  // Integral term with anti-windup
  integral_ += error * dt;
  //integral_ = std::clamp(integral_, -1.5, 1.5); // Tighter anti-windup for stability
  
  // Derivative term with low-pass filtering to reduce noise
  double raw_derivative = (error - prev_error_) / dt;
  static double filtered_derivative = 0.0;
  filtered_derivative = 0.8 * filtered_derivative + 0.2 * raw_derivative; // Low-pass filter
  double derivative = filtered_derivative;
  prev_error_ = error;
  
  // Full PID calculation
  double pid_output = Kp_ * error + Ki_ * integral_ + Kd_ * derivative;
  
  // Feedforward term for better tracking
  double ff_output = target_velocity_ * 0.2;
  
  double control_output = pid_output + ff_output;
  
  // Convert to PWM and direction
  if (fabs(control_output) > 0.01) {
    int direction = (control_output >= 0) ? 1 : 0; // Fixed direction logic: positive velocity = dir 1, negative velocity = dir 0
    int duty_cycle = static_cast<int>(fabs(control_output) * 120.0); // Increased gain for better response
    duty_cycle = std::clamp(duty_cycle, 15, 255);
    
    gpioWrite(dir_pin_, direction);
    gpioPWM(pwm_pin_, duty_cycle);
    
    // std::cout << "[Motor] target=" << target_velocity_ << " current=" << current 
    //           << " error=" << error << " P=" << Kp_*error << " I=" << Ki_*integral_ 
    //           << " D=" << Kd_*derivative << " output=" << control_output 
    //           << " PWM=" << duty_cycle << " DIR=" << direction << std::endl;
  } else {
    // Stop motor
    gpioWrite(dir_pin_, 0);
    gpioPWM(pwm_pin_, 0);
    //std::cout << "[Motor] STOPPED" << std::endl;
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
  std::cout << "[Motor PWM:" << pwm_pin_ << "] STOP called" << std::endl;
  
  // Stop motor exactly like working test:
  // 1. Set PWM to 0
  gpioPWM(pwm_pin_, 0);
  
  // 2. Set direction to 0 (safe state)
  gpioWrite(dir_pin_, 0);
  
  // 3. Clear target velocity
  target_velocity_ = 0.0;
  current_velocity_ = 0.0;
  
  std::cout << "[Motor PWM:" << pwm_pin_ << "] STOPPED (PWM=0, DIR=0)" << std::endl;
}

void ServoCityMotor::setMotorOutput(int pwm) {
  gpioServo(pwm_pin_, pwm);
}