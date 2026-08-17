#pragma once

#include <cstdint>
#include <atomic>

#include "halx/peripheral.hpp"

/**
 * @brief 直交エンコーダ(2逓倍/4逓倍カウント)のラッパー。
 * `update()` を一定周期`dt`で呼び出して出力軸の速度・位置を更新する。
 * @tparam Handle エンコーダに使うタイマーのハンドル
 *
 * @code{.cpp}
 * #include "encoder.hpp"
 *
 * extern TIM_HandleTypeDef htim1; // エンコーダ
 *
 * constexpr float DT = 0.01f; // 100 Hz
 * Encoder<&htim1> encoder1(2048, 2.0f, DT);
 *
 * extern "C" void app_main() {
 *   encoder1.start();
 *
 *   // 100 Hzの周期割り込み等から呼び出す
 *   encoder1.update();
 *   float rps = encoder1.get_rps();
 * }
 * @endcode
 */
template <TIM_HandleTypeDef *Handle> class Encoder {
public:
  /**
   * @brief コンストラクタ。
   * @param ppr エンコーダの1回転あたりパルス数
   * @param gear_ratio ギア比(出力軸1回転あたりのモーター軸回転数)
   * @param dt `update()` を呼び出す周期[秒]
   */
  Encoder(uint32_t ppr, float gear_ratio, float dt)
      : ppr_{ppr}, gear_ratio_{gear_ratio}, dt_{dt} {}

  /**
   * @brief エンコーダのカウントを開始する。
   */
  void start() { halx::peripheral::ST_TIM<Handle>::start_encoder(); }

  /**
   * @brief エンコーダカウンタを読み取ってリセットし、出力軸の回転速度・位置を更新する。
   * コンストラクタに渡した周期`dt`ごとに呼び出すこと。
   */
  void update() {
    int16_t count = halx::peripheral::ST_TIM<Handle>::get_counter();
    halx::peripheral::ST_TIM<Handle>::set_counter(0);

    float delta = count / (ppr_ * 4.0f) / gear_ratio_;
    rps_ = delta / dt_;
    position_ += delta;
  }

  /**
   * @brief 直近の `update()` で計算した出力軸の回転速度を取得する。
   * @return 回転速度[rps]
   */
  float get_rps() const { return rps_; }

  /**
   * @brief `update()` 呼び出しごとに積算された出力軸の位置を取得する。
   * @return 位置[回転数]
   */
  float get_position() const { return position_; }

private:
  uint32_t ppr_;
  float gear_ratio_;
  float dt_;
  std::atomic<float> rps_ = 0.0f;
  std::atomic<float> position_ = 0.0f;
};
