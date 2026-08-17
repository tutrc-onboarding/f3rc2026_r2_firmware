#pragma once

#include <algorithm>
#include <optional>

/**
 * @brief `PIDController` に渡すゲイン・クリップ設定。
 */
struct PIDParameters {
  float kp = 0.0;    ///< 比例ゲイン
  float ki = 0.0;    ///< 積分ゲイン
  float kd = 0.0;    ///< 微分ゲイン
  std::optional<float> output_upper_limit = std::nullopt;    ///< 出力の絶対値上限(未設定ならクリップしない)
  std::optional<float> integral_upper_limit = std::nullopt;  ///< 積分項(内部状態)の絶対値上限(アンチワインドアップ用、未設定ならクリップしない)
};

/**
 * @brief 位置型PIDコントローラ。`solve()` を一定周期`dt`で呼び出して使う。
 *
 * @code{.cpp}
 * #include "pid_controller.hpp"
 *
 * constexpr float DT = 0.01f; // 100 Hz
 * constexpr PIDParameters PARAMS{
 *     .kp = 0.0f,
 *     .ki = 0.5858f,
 *     .output_upper_limit = 0.5f,
 *     .integral_upper_limit = 0.5f / 0.5858f,
 * };
 * PIDController motor1_pid(PARAMS, DT);
 *
 * // 100 Hzの周期割り込み等から呼び出す
 * float target_rps = 2.0f;
 * float output = motor1_pid.solve(target_rps - motor1.get_rps());
 * motor1.set_output(output);
 * @endcode
 */
class PIDController {
public:
  /**
   * @brief コンストラクタ。
   * @param params ゲイン・クリップ設定
   * @param dt `solve()` を呼び出す周期[秒]
   */
  PIDController(const PIDParameters &params, float dt)
      : params_{params}, dt_{dt} {}

  /**
   * @brief 誤差を1周期分進めて操作量を計算する。
   * @param error 目標値との誤差(目標 - 現在値)
   * @return 計算された操作量(output_upper_limitが設定されていればクリップ済み)
   */
  float solve(float error) {
    integral_ += error * dt_;

    // 積分項クリップ
    if (params_.integral_upper_limit) {
      integral_ = std::clamp(integral_, -*params_.integral_upper_limit,
                             *params_.integral_upper_limit);
    }

    float output = params_.kp * error + params_.ki * integral_ +
                   params_.kd * (error - prev_error_) / dt_;
    prev_error_ = error;

    // 出力クリップ
    if (params_.output_upper_limit) {
      output = std::clamp(output, -*params_.output_upper_limit,
                          *params_.output_upper_limit);
    }

    return output;
  }

private:
  PIDParameters params_;
  float dt_;
  float prev_error_ = 0;
  float integral_ = 0;
};
