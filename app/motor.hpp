#pragma once

#include <algorithm>
#include <cstdint>

#include "halx/driver/gpio.hpp"
#include "halx/peripheral.hpp"

/**
 * @brief PWM出力・enableピンをまとめたDCモーター1個分のラッパー。
 * @tparam Handle PWM出力に使うタイマーのハンドル
 *
 * @code{.cpp}
 * #include "motor.hpp"
 *
 * extern TIM_HandleTypeDef htim15; // PWM
 *
 * halx::driver::GPIO motor1_pin(Motor1_GPIO_Port, Motor1_Pin);
 * Motor<&htim15> motor1(TIM_CHANNEL_1, motor1_pin);
 *
 * extern "C" void app_main() {
 *   motor1.start();
 *   motor1.set_output(0.2f); // -1.0〜1.0
 * }
 * @endcode
 */
template <TIM_HandleTypeDef *Handle> class Motor {
public:
  /**
   * @brief コンストラクタ。
   * @param pwm_channel PWM出力チャンネル(`TIM_CHANNEL_1`など)
   * @param motor_pin モータードライバのenable/directionピン
   */
  Motor(uint32_t pwm_channel, halx::driver::GPIO &motor_pin)
      : pwm_channel_{pwm_channel}, motor_pin_{motor_pin} {}

  /**
   * @brief 出力0でPWMを開始し、モータードライバを有効化する。
   */
  void start() {
    set_output(0.0f);
    halx::peripheral::ST_TIM<Handle>::start_pwm(pwm_channel_);
    motor_pin_.write(halx::driver::GPIOState::HIGH);
  }

  /**
   * @brief PWM出力を設定する。
   * @param output 出力(-1.0〜1.0にクランプされ、0〜100%duty比に線形変換される)
   */
  void set_output(float output) {
    float duty =
        std::clamp((std::clamp(output, -1.0f, 1.0f) + 1.0f) * 0.5f, 0.0f, 1.0f);
    uint32_t period = halx::peripheral::ST_TIM<Handle>::get_autoreload();
    halx::peripheral::ST_TIM<Handle>::set_compare(
        pwm_channel_, static_cast<uint32_t>(period * duty));
  }

private:
  uint32_t pwm_channel_;
  halx::driver::GPIO &motor_pin_;
};
