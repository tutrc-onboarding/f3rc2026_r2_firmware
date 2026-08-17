#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>

#include <halx/driver/uart_base.hpp>

/// PS3コントローラのスティック軸。
enum class PS3Axis {
  LEFT_X,
  LEFT_Y,
  RIGHT_X,
  RIGHT_Y,
};

/// PS3コントローラのボタン。
enum class PS3Key {
  UP,
  DOWN,
  RIGHT,
  LEFT,
  TRIANGLE,
  CROSS,
  CIRCLE,
  SQUARE = 8,
  L1,
  L2,
  R1,
  R2,
  START,
  SELECT,
};

/**
 * @brief UART経由でPS3コントローラの受信機からスティック・ボタン状態を読み取る。
 * `update()` をメインループ等から繰り返し呼び出して状態を最新化する。
 *
 * @code{.cpp}
 * #include "ps3.hpp"
 * #include <halx/driver/uart_it.hpp>
 *
 * extern UART_HandleTypeDef huart5;
 * halx::driver::UART_IT<&huart5> uart5;
 * PS3 ps3(uart5);
 *
 * extern "C" void app_main() {
 *   uart5.start();
 *
 *   while (true) {
 *     ps3.update();
 *
 *     float x = ps3.get_axis(PS3Axis::LEFT_X);
 *     if (ps3.get_key_down(PS3Key::CROSS)) {
 *       // ×ボタンが押された瞬間の処理
 *     }
 *   }
 * }
 * @endcode
 */
class PS3 {
public:
  /**
   * @brief コンストラクタ。
   * @param uart 受信機と接続されたUART
   */
  PS3(halx::driver::UARTBase &uart) : uart_{uart} {}

  /**
   * @brief 受信バッファにあるメッセージを全て読み取り、スティック・ボタン状態を更新する。
   */
  void update() {
    keys_prev_ = keys_;

    while (receive_message()) {
      if (test_checksum(buf_)) {
        keys_ = (buf_[1] << 8) | buf_[2];
        if ((keys_ & 0x03) == 0x03) {
          keys_ &= ~0x03;
          keys_ |= 1 << 13;
        }
        if ((keys_ & 0x0C) == 0x0C) {
          keys_ &= ~0x0C;
          keys_ |= 1 << 14;
        }
        for (size_t i = 0; i < 4; ++i) {
          axes_[i] = (static_cast<float>(buf_[i + 3]) - 64) / 64;
        }
      }
      buf_.fill(0);
    }
  }

  /**
   * @brief 指定した軸の値を取得する。
   * @param axis 取得する軸
   * @return 軸の値(中央0、概ね-1.0〜1.0)
   */
  float get_axis(PS3Axis axis) { return axes_[std::to_underlying(axis)]; }

  /**
   * @brief 指定したボタンが現在押されているかを取得する。
   * @param key 対象のボタン
   * @return 押されていればtrue
   */
  bool get_key(PS3Key key) {
    return (keys_ & (1 << std::to_underlying(key))) != 0;
  }

  /**
   * @brief 直前の `update()` との比較で、指定したボタンが押された瞬間かを取得する。
   * @param key 対象のボタン
   * @return このフレームで押された瞬間ならtrue
   */
  bool get_key_down(PS3Key key) {
    return ((keys_ ^ keys_prev_) & keys_ & (1 << std::to_underlying(key))) != 0;
  }

  /**
   * @brief 直前の `update()` との比較で、指定したボタンが離された瞬間かを取得する。
   * @param key 対象のボタン
   * @return このフレームで離された瞬間ならtrue
   */
  bool get_key_up(PS3Key key) {
    return ((keys_ ^ keys_prev_) & keys_prev_ &
            (1 << std::to_underlying(key))) != 0;
  }

private:
  halx::driver::UARTBase &uart_;
  std::array<uint8_t, 8> buf_{};
  std::array<float, 4> axes_{};
  uint16_t keys_ = 0;
  uint16_t keys_prev_ = 0;

  /// ヘッダ(0x80)を探して8バイトのメッセージを1件受信する。
  bool receive_message() {
    // ヘッダ探す
    for (size_t i = 0; i < 8; ++i) {
      if (buf_[0] == 0x80) {
        break;
      }
      if (!uart_.read(&buf_[0], 1, 0)) {
        return false;
      }
    }
    if (buf_[0] != 0x80) {
      return false;
    }

    // メッセージの残りの部分を受信
    if (!uart_.read(&buf_[1], 7, 0)) {
      return false;
    }
    return true;
  }

  /// 受信メッセージのチェックサムを検証する。
  static inline bool test_checksum(const std::array<uint8_t, 8> &msg) {
    uint8_t checksum = 0;
    for (size_t i = 1; i < 7; ++i) {
      checksum += msg[i];
    }
    return (checksum & 0x7F) == msg[7];
  }
};
