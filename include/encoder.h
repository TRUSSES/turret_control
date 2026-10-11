#ifndef ENCODER_H_
#define ENCODER_H_

#include <pigpio.h>
#include <atomic>
#include <iostream>
#include <mutex>
#include <yaml-cpp/yaml.h>

class Encoder {
 public:
  Encoder(int cs_pin, int clk_pin, int do_pin, int encoder_max_value = 1023, int offset = 0);
  
  // Overloaded constructor using YAML config.
  explicit Encoder(const YAML::Node &node);

  int Read() const;
  void Update();
  int GetCount() const;
  void ResetCount();
  void SetOffset(int offset);

 private:
  int ReadUnlocked() const;
  mutable std::mutex mutex_;
  int cs_pin_;
  int clk_pin_;
  int do_pin_;
  int encoder_max_value_;
  int offset_;
  int previous_value_;
  std::atomic<int> encoder_count_;
};

#endif  // ENCODER_H_
