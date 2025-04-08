#ifndef ENCODER_H_
#define ENCODER_H_

#include <pigpio.h>
#include <iostream>

class Encoder {
 public:
  // Constructs an Encoder with the specified chip-select, clock, and data GPIO pins.
  // encoder_max_value is typically 1023 and offset is used for encoders that need calibration (e.g., turret).
  Encoder(int cs_pin, int clk_pin, int do_pin, int encoder_max_value = 1023, int offset = 0);

  // Delete copy constructor and assignment operator.
  Encoder(const Encoder&) = delete;
  Encoder& operator=(const Encoder&) = delete;

  // Reads the encoder value by communicating with the encoder hardware,
  // applies bit-shifting and offset correction, and returns the processed value.
  int Read() const;

  // Updates the cumulative encoder count by comparing the current reading with the previous one,
  // adjusting for roll-over.
  void Update();

  // Returns the current cumulative encoder count.
  int GetCount() const;

  // Resets the encoder count to zero and updates the previous reading.
  void ResetCount();

  // Sets the offset used to adjust the raw encoder reading.
  void SetOffset(int offset);

 private:
  int cs_pin_;
  int clk_pin_;
  int do_pin_;
  int encoder_max_value_;
  int offset_;
  int previous_value_;
  int encoder_count_;
};

#endif  // ENCODER_H_
