#include "encoder.h"

Encoder::Encoder(int cs_pin, int clk_pin, int do_pin, int encoder_max_value, int offset)
    : cs_pin_(cs_pin),
      clk_pin_(clk_pin),
      do_pin_(do_pin),
      encoder_max_value_(encoder_max_value),
      offset_(offset),
      encoder_count_(0) {
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
  if (value > static_cast<uint16_t>(encoder_max_value_)) {
    std::cerr << "Warning: Encoder value out of range: " << value << std::endl;
    value &= 0x03FF;
  }
  int processed_value = static_cast<int>(value);
  if (offset_ != 0) {
    processed_value -= offset_;
    if (processed_value < 0) {
      processed_value += (encoder_max_value_ + 1);
    } else if (processed_value > encoder_max_value_) {
      processed_value -= (encoder_max_value_ + 1);
    }
  }
  return processed_value;
}

void Encoder::Update() {
  int current_value = Read();
  int difference = current_value - previous_value_;
  if (difference > encoder_max_value_ / 2) {
    difference -= (encoder_max_value_ + 1);
  } else if (difference < -encoder_max_value_ / 2) {
    difference += (encoder_max_value_ + 1);
  }
  encoder_count_ -= difference;
  previous_value_ = current_value;
}

int Encoder::GetCount() const {
  return encoder_count_;
}

void Encoder::ResetCount() {
  encoder_count_ = 0;
  previous_value_ = Read();
}

void Encoder::SetOffset(int offset) {
  offset_ = offset;
}
