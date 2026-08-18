#pragma once

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "feetech_servo.hpp"

class FeetechPositionControl {
public:
  FeetechPositionControl(halx::driver::UARTBase &uart, uint8_t id, float initial_position, uint16_t min_angle_limit,
                         uint16_t max_angle_limit)
      : servo_{uart, id}, position_target_{initial_position}, min_angle_limit_{min_angle_limit},
        max_angle_limit_{max_angle_limit} {}

  void start() {
    while (!servo_.ping()) {
    }

    uint8_t buf[2];
    std::memcpy(buf, &min_angle_limit_, sizeof(buf));
    servo_.write_data(0x09, buf, sizeof(buf));
    std::memcpy(buf, &max_angle_limit_, sizeof(buf));
    servo_.write_data(0x0B, buf, sizeof(buf));

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
  std::atomic<float> position_target_;
  uint16_t min_angle_limit_;
  uint16_t max_angle_limit_;

  void read_position() {
    int16_t raw_position;
    if (servo_.get_position(raw_position)) {
      position_ = static_cast<float>(raw_position) / 4096.0f;
    }
  }

  void write_position(float position) { servo_.set_position(static_cast<int16_t>(position * 4096.0f)); }
};
