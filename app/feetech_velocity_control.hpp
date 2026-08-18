#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>

#include "feetech_servo.hpp"

class FeetechVelocityControl {
public:
  FeetechVelocityControl(halx::driver::UARTBase &uart, uint8_t id) : servo_{uart, id} {}

  void start() {
    while (!servo_.ping()) {
    }
    servo_.control_mode(1);
    servo_.enable_torque(1);
  }

  float get_position() {
    float position = position_;
    return std::isnan(position) ? 0.0f : position;
  }

  float get_velocity() { return velocity_; }

  void set_velocity(float velocity) { velocity_target_ = velocity; }

  void set_zero_position() { position_ = 0.0f; }

  void update() {
    read_position();
    read_velocity();

    float position = position_;
    if (std::isnan(position)) {
      position = 0.0f;
      raw_position_prev_ = raw_position_;
    }
    int16_t delta = raw_position_ - raw_position_prev_;
    if (delta < -(4096 / 2)) {
      delta += 4096;
    } else if (delta > (4096 / 2)) {
      delta -= 4096;
    }
    position += static_cast<float>(delta) / 4096.0f;
    raw_position_prev_ = raw_position_;
    position_ = position;

    write_velocity(velocity_target_);
  }

private:
  FeetechServo servo_;
  int16_t raw_position_ = 0;
  int16_t raw_position_prev_ = 0;
  int16_t raw_velocity_ = 0;
  std::atomic<float> position_ = NAN;
  std::atomic<float> velocity_ = 0.0f;
  std::atomic<float> velocity_target_ = 0.0f;

  void read_position() {
    int16_t raw_position;
    if (servo_.get_position(raw_position)) {
      raw_position_ = raw_position;
    }
  }

  void read_velocity() {
    int16_t raw_velocity;
    if (servo_.get_velocity(raw_velocity)) {
      raw_velocity_ = raw_velocity;
    }
    velocity_ = static_cast<float>(raw_velocity_) / 4096.0f;
  }

  void write_velocity(float velocity) { servo_.set_velocity(static_cast<int16_t>(velocity * 4096.0f)); }
};
