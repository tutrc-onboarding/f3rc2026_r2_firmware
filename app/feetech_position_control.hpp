#pragma once

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "feetech_servo.hpp"

class FeetechPositionControl {
public:
  FeetechPositionControl(halx::driver::UARTBase &uart, uint8_t id, int16_t initial_position)
      : servo_{uart, id}, position_target_{initial_position} {}

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
  std::atomic<int16_t> position_ = 0;
  std::atomic<int16_t> position_target_;
  uint16_t min_angle_limit_;
  uint16_t max_angle_limit_;

  void read_position() {
    int16_t position;
    if (servo_.get_position(position)) {
      position_ = position;
    }
  }

  void write_position(int16_t position) { servo_.set_position(position); }
};
