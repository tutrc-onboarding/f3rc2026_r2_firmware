#pragma once

#include <atomic>
#include <cstdint>

#include "feetech_servo.hpp"

class FeetechPositionControl {
public:
  FeetechPositionControl(halx::driver::UARTBase &uart, uint8_t id) : servo_{uart, id} {}

  void start() {
    while (!servo_.ping()) {
    }
    servo_.control_mode(0);
    servo_.enable_torque(1);
  }

  float get_position() { return position_; }

  void set_position(float position) { position_target_ = position; }

  void update() {
    read_position();
    write_position(position_target_);
  }

private:
  FeetechServo servo_;
  std::atomic<float> position_ = 0.0f;
  std::atomic<float> position_target_ = 0.0f;

  void read_position() {
    int16_t raw_position;
    if (servo_.get_position(raw_position)) {
      position_ = static_cast<float>(raw_position) / 4096.0f;
    }
  }

  void write_position(float position) { servo_.set_position(static_cast<int16_t>(position * 4096.0f)); }
};
