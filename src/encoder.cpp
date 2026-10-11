#include "encoder.h"

Encoder::Encoder(int cs_pin, int clk_pin, int do_pin, int encoder_max_value, int offset)
    : cs_pin_(cs_pin),
      clk_pin_(clk_pin),
      do_pin_(do_pin),
      encoder_max_value_(encoder_max_value),
      offset_(offset),
      encoder_count_(0) {
  // Restore the configured directions even if a previous process left a pin
  // in another mode. Encoder data must remain an input during CAN traffic.
  gpioSetMode(cs_pin_, PI_OUTPUT);
  gpioSetMode(clk_pin_, PI_OUTPUT);
  gpioSetMode(do_pin_, PI_INPUT);
  gpioWrite(cs_pin_, PI_HIGH);
  gpioWrite(clk_pin_, PI_LOW);
  previous_value_ = Read();
}

Encoder::Encoder(const YAML::Node &node)
    : Encoder(node["cs_pin"].as<int>(),
              node["clk_pin"].as<int>(),
              node["do_pin"].as<int>(),
              node["encoder_max_value"] ? node["encoder_max_value"].as<int>() : 1023,
              node["offset"] ? node["offset"].as<int>() : 0) {
}

int Encoder::Read() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return ReadUnlocked();
}

int Encoder::ReadUnlocked() const {
  uint16_t value = 0;
  gpioWrite(cs_pin_, PI_LOW);
  gpioDelay(5);
  for (int i = 0; i < 16; i++) {
    gpioWrite(clk_pin_, PI_HIGH);
    gpioDelay(5);
    value <<= 1;
    if (gpioRead(do_pin_)) {
      value |= 1;
    }
    gpioWrite(clk_pin_, PI_LOW);
    gpioDelay(5);
  }
  gpioWrite(cs_pin_, PI_HIGH);
  value >>= 6;

  // Check if the value is in the expected 10-bit range (0-1023)
  if (value > 1023) {
    std::cerr << "Warning: Encoder value out of range: " << value << std::endl;
    value &= 0x03FF; // Mask to keep it within 10 bits
  }

  return value;
}

void Encoder::Update() {
  std::lock_guard<std::mutex> lock(mutex_);
  int current_value = ReadUnlocked();
  int difference = current_value - previous_value_;

  // Handle wrap-around based on the direction of movement
  // This matches the reference implementation logic
  if (difference > encoder_max_value_ / 2) {
    // Positive wrap-around (e.g., from 1023 to 0)
    difference -= (encoder_max_value_ + 1);
  } else if (difference < -encoder_max_value_ / 2) {
    // Negative wrap-around (e.g., from 0 to 1023)
    difference += (encoder_max_value_ + 1);
  }

  // Update continuous encoder count (subtract like reference implementation)
  encoder_count_.fetch_sub(difference, std::memory_order_relaxed);
  previous_value_ = current_value;
}

int Encoder::GetCount() const {
  return encoder_count_.load(std::memory_order_relaxed);
}

void Encoder::ResetCount() {
  std::lock_guard<std::mutex> lock(mutex_);
  previous_value_ = ReadUnlocked();
  encoder_count_.store(0, std::memory_order_relaxed);
}

void Encoder::SetOffset(int offset) {
  std::lock_guard<std::mutex> lock(mutex_);
  offset_ = offset;
}
