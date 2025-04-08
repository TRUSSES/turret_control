#include "servo_city_motor.h"
#include <algorithm>

ServoCityMotor* ServoCityMotor::instance = nullptr;

ServoCityMotor::ServoCityMotor(int pwm_pin, int dir_pin, int enc_a, int enc_b, int enable_pin)
    : pwm_pin_(pwm_pin), dir_pin_(dir_pin), enc_a_(enc_a), enc_b_(enc_b), enable_pin_(enable_pin) {
  if(gpioInitialise() < 0) {
    std::cerr << "pigpio initialization failed" << std::endl;
    exit(1);
  }
  gpioSetMode(pwm_pin_, PI_OUTPUT);
  gpioSetMode(dir_pin_, PI_OUTPUT);
  gpioSetPWMfrequency(pwm_pin_, 10000);
  gpioWrite(dir_pin_, 0);
  gpioSetMode(enable_pin_, PI_OUTPUT);
  gpioSetMode(enc_a_, PI_INPUT);
  gpioSetMode(enc_b_, PI_INPUT);
  gpioSetPullUpDown(enc_a_, PI_PUD_UP);
  gpioSetPullUpDown(enc_b_, PI_PUD_UP);
  gpioGlitchFilter(enc_a_, 1000);
  gpioGlitchFilter(enc_b_, 1000);
  gpioSetISRFunc(enc_a_, EITHER_EDGE, 0, encoderISR);
  gpioSetISRFunc(enc_b_, EITHER_EDGE, 0, encoderISR);
  instance = this;
  last_update_ = std::chrono::steady_clock::now();
  last_encoder_time_ = std::chrono::steady_clock::now();
}

ServoCityMotor::ServoCityMotor(const YAML::Node &node)
    : ServoCityMotor(node["pwm_pin"].as<int>(),
                     node["dir_pin"].as<int>(),
                     node["enc_a"].as<int>(),
                     node["enc_b"].as<int>(),
                     node["enable_pin"].as<int>()) {
}

ServoCityMotor::~ServoCityMotor() {
  gpioTerminate();
}

void ServoCityMotor::setTargetVelocity(double target_rad_per_sec) {
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
  if(integral_ < -1.0)
    integral_ = -1.0;
  else if(integral_ > 1.0)
    integral_ = 1.0;
  double derivative = (error - prev_error_) / dt;
  prev_error_ = error;
  
  double output = Kp_ * error + Ki_ * integral_ + Kd_ * derivative;
  if(output < -1.0)
    output = -1.0;
  else if(output > 1.0)
    output = 1.0;
  
  output_filter_ = (1.0 - filter_gain_) * output_filter_ + filter_gain_ * output;
  
  if(fabs(output_filter_) > 0.05) {
    gpioWrite(dir_pin_, output_filter_ > 0 ? 1 : 0);
    gpioPWM(pwm_pin_, static_cast<int>(fabs(output_filter_) * 255));
  } else {
    gpioPWM(pwm_pin_, 0);
  }
  
  auto now_enc = std::chrono::steady_clock::now();
  double dt_enc = std::chrono::duration<double>(now_enc - last_encoder_time_).count();
  if(dt_enc > 0.01) {
    int counts = encoder_count.exchange(0);
    double delta_rad = countsToRadians(counts);
    double raw_velocity = delta_rad / dt_enc;
    
    velocity_filter_ = (1.0 - velocity_filter_gain_) * velocity_filter_ 
                     + velocity_filter_gain_ * raw_velocity;
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
  gpioWrite(enable_pin_, 0);
  integral_ = 0.0;
  prev_error_ = 0.0;
}

void ServoCityMotor::setMotorOutput(int pwm) {
  gpioServo(pwm_pin_, pwm);
}
