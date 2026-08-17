// feetechを使いやすくするclass　連続回転、位置原点リセットに対応している
#pragma once

#include <cstdint>

#include "feetech_servo.hpp"
#include "pid_controller.hpp"

class FeetechContinuous {
public:
  FeetechContinuous(uint8_t id, halx::driver::UARTBase &uart,
                    bool set_direction_state = true, float dt = 0.01f)
      : servo_(id, uart),
        pos_pid_(PIDParameters{.kp = POS_CONTROL_P_GAIN,
                               .output_upper_limit = POS_CONTROL_VEL_LIMIT},
                 dt) {
    first_state_ = true;
    if (set_direction_state) {
      direction_ = 1;
    } else {
      direction_ = -1;
    }
    ftc_present_position_ = 0;
    ftc_present_position_continuous_ = 0;
    ftc_present_position_old_ = 0;
    ftc_present_velocity_ = 0;
    pos_now_ = 0.0;
    vel_now_ = 0.0;
    position_target_ = 0.0;
    velocity_target_ = 0.0;
    target_mode_ = 0;
  }

  // トルクのON/OFFを設定する関数
  void enable_torque(bool value) { servo_.enable_torque(value ? 1 : 0); }

  // 制御モードを設定する関数
  void control_mode(uint8_t value) { servo_.control_mode(value); }

  // 現在位置を取得する関数(通信はしない)
  float get_position() {
    pos_now_ = ftc_pos_to_pos(ftc_present_position_continuous_);
    return pos_now_;
  }

  // 現在速度を取得する関数(通信はしない)
  float get_velocity() {
    vel_now_ = ftc_vel_to_vel(ftc_present_velocity_);
    return vel_now_;
  }

  // 目標速度を指定する関数
  void set_velocity_target(float set_velocity_target) {
    target_mode_ = 0;
    velocity_target_ = set_velocity_target;
  }

  // 目標位置を指定する関数
  void set_position_target(float set_position_target) {
    target_mode_ = 1;
    position_target_ = set_position_target;
  }

  // 現在位置(連続)を0セットする関数(通信はしない)
  void set_zero_position() { ftc_present_position_continuous_ = 0; }

  // 常にメインループ(main_thread.cppのwhile(1))内で実行する関数
  void update() {
    read_position();
    read_velocity();

    // 位置を連続で取るための処理
    if (first_state_) { // 初回のみ現在値を代入
      ftc_present_position_continuous_ = 0;
      ftc_present_position_old_ = ftc_present_position_;
      first_state_ = false;
    }
    int16_t delta = ftc_present_position_ - ftc_present_position_old_;
    if (delta < -(4096 / 2)) {
      delta += 4096;
    } else if (delta > (4096 / 2)) {
      delta -= 4096;
    }
    ftc_present_position_continuous_ += delta;
    ftc_present_position_old_ = ftc_present_position_;

    if (target_mode_ == 0) {
      send_velocity_target(velocity_target_);
    } else if (target_mode_ == 1) {
      send_velocity_target(pos_pid_.solve(position_target_ - pos_now_));
    }
  }

private:
  static constexpr float POS_CONTROL_P_GAIN = 5.0f; // 位置制御のPゲイン[1/s]
  static constexpr float POS_CONTROL_VEL_LIMIT =
      250.0f; // 位置制御での速度制限[deg/s]

  FeetechServo servo_;

  bool first_state_;
  int16_t ftc_present_position_;
  int16_t
      ftc_present_position_continuous_; // 0°と360°の境界で振り切れないように修正した位置
  int16_t ftc_present_position_old_;
  int16_t ftc_present_velocity_;
  int direction_;         // 1 or -1
  float pos_now_;         // [deg]
  float vel_now_;         // [deg/s]
  float position_target_; // [deg]
  float velocity_target_; // [deg/s]

  bool target_mode_;      // 0:速度制御モード 1:位置制御モード
  PIDController pos_pid_; // 位置制御(P制御)に使うPIDコントローラ

  // feetechから取得した角度を[deg]に変換する関数
  float ftc_pos_to_pos(int16_t ftc_pos) {
    return (float)direction_ * (float)ftc_pos * 360.0 / 4096.0;
  }

  // feetechから取得した角速度を[deg/s]に変換する関数
  float ftc_vel_to_vel(int16_t ftc_vel) {
    return (float)direction_ * (float)ftc_vel * 360.0 / 4096.0;
  }

  // 角速度[deg/s]をfeetech用の角速度に変換する関数
  int16_t vel_to_ftc_vel(float vel) {
    return direction_ * (int16_t)(vel * 4096.0 / 360.0);
  }

  // 現在位置を取得する関数(通信する)
  float read_position() {
    int16_t ftc_pos_now;
    if (servo_.get_position(ftc_pos_now)) {
      ftc_present_position_ = ftc_pos_now;
    }

    pos_now_ = ftc_pos_to_pos(ftc_present_position_continuous_);
    return pos_now_;
  }

  // 現在速度を取得する関数(通信する)
  float read_velocity() {
    int16_t ftc_vel_now;
    if (servo_.get_velocity(ftc_vel_now)) {
      ftc_present_velocity_ = ftc_vel_now;
    }
    vel_now_ = ftc_vel_to_vel(ftc_present_velocity_);
    return vel_now_;
  }

  // 目標速度を指定する関数(通信する)
  void send_velocity_target(float send_velocity_target) {
    int16_t ftc_vel_target = vel_to_ftc_vel(send_velocity_target);
    servo_.set_velocity(ftc_vel_target);
  }
};
