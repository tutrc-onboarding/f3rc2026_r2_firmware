#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include <halx/driver/uart_base.hpp>

/**
 * @brief Feetech社製シリアルサーボ(半二重UART, SCSプロトコル)のドライバ。
 *
 * @code{.cpp}
 * #include "feetech_servo.hpp"
 * #include <halx/driver/uart_it.hpp>
 *
 * extern UART_HandleTypeDef huart1;
 * halx::driver::UART_IT<&huart1> uart1;
 * FeetechServo servo(1, uart1);
 *
 * extern "C" void app_main() {
 *   uart1.start();
 *
 *   if (servo.ping()) {
 *     servo.control_mode(0);   // 位置制御モード
 *     servo.enable_torque(1);
 *     servo.set_position(1000);
 *   }
 * }
 * @endcode
 */
class FeetechServo {
public:
  /**
   * @brief コンストラクタ。
   * @param id サーボのID
   * @param uart サーボと接続されたUART(半二重、echo受信を前提とする)
   */
  FeetechServo(uint8_t id, halx::driver::UARTBase &uart)
      : id_{id}, uart_{uart} {}

  /**
   * @brief 直近の応答パケットに含まれていたエラーコードを取得する。
   * @return エラーコード(0ならエラーなし)
   */
  uint8_t get_error() const { return error_; }

  /**
   * @brief サーボの生存確認を行う。
   * @return 応答があればtrue
   */
  bool ping() {
    uint8_t checksum = 0;
    uart_.clear();
    write_byte(0xFF);
    write_byte(0xFF);
    checksum += write_byte(id_);
    checksum += write_byte(2);
    checksum += write_byte(0x01);
    write_byte(~checksum);
    return check_reply(nullptr, 0);
  }

  /**
   * @brief サーボのメモリを読み取る。
   * @param addr 読み取り開始アドレス
   * @param data 読み取ったデータの格納先
   * @param size 読み取りバイト数
   * @return 成功した場合true
   */
  bool read_data(uint8_t addr, uint8_t *data, size_t size) {
    uint8_t checksum = 0;
    uart_.clear();
    write_byte(0xFF);
    write_byte(0xFF);
    checksum += write_byte(id_);
    checksum += write_byte(4);
    checksum += write_byte(0x02);
    checksum += write_byte(addr);
    checksum += write_byte(size);
    write_byte(~checksum);
    return check_reply(data, size);
  }

  /**
   * @brief サーボのメモリに書き込む。
   * @param addr 書き込み開始アドレス
   * @param data 書き込むデータ
   * @param size 書き込みバイト数
   * @return 成功した場合true
   */
  bool write_data(uint8_t addr, const uint8_t *data, size_t size) {
    uint8_t checksum = 0;
    uart_.clear();
    write_byte(0xFF);
    write_byte(0xFF);
    checksum += write_byte(id_);
    checksum += write_byte(size + 3);
    checksum += write_byte(0x03);
    checksum += write_byte(addr);
    for (size_t i = 0; i < size; ++i) {
      checksum += write_byte(data[i]);
    }
    write_byte(~checksum);
    return check_reply(nullptr, 0);
  }

  /**
   * @brief 制御モード(位置/速度など)を設定する。
   * @param value モード値
   * @return 成功した場合true
   */
  bool control_mode(uint8_t value) {
    return write_data(0x21, &value, sizeof(value));
  }

  /**
   * @brief トルクのON/OFFを設定する。
   * @param value 0でOFF、非0でON
   * @return 成功した場合true
   */
  bool enable_torque(uint8_t value) {
    return write_data(0x28, &value, sizeof(value));
  }

  /**
   * @brief 目標位置を設定する。
   * @param value 目標位置(符号付き、内部で符号ビット表現に変換される)
   * @return 成功した場合true
   */
  bool set_position(int16_t value) {
    value = std::clamp<int16_t>(value, -32766, 32766);
    if (value < 0) {
      value = std::abs(value) | 0x8000;
    }
    uint8_t buf[2];
    std::memcpy(buf, &value, sizeof(buf));
    return write_data(0x2A, buf, sizeof(buf));
  }

  /**
   * @brief 目標速度を設定する。
   * @param value 目標速度(符号付き、内部で符号ビット表現に変換される)
   * @return 成功した場合true
   */
  bool set_velocity(int16_t value) {
    value = std::clamp<int16_t>(value, -32766, 32766);
    if (value < 0) {
      value = std::abs(value) | 0x8000;
    }
    uint8_t buf[2];
    std::memcpy(buf, &value, sizeof(buf));
    return write_data(0x2E, buf, sizeof(buf));
  }

  /**
   * @brief 現在位置を取得する。
   * @param value 取得した位置の格納先
   * @return 成功した場合true
   */
  bool get_position(int16_t &value) {
    uint8_t buf[2];
    if (!read_data(0x38, buf, sizeof(buf))) {
      return false;
    }
    std::memcpy(&value, buf, sizeof(value));
    if ((value & 0x8000) != 0) {
      value = -(value & 0x7FFF);
    }
    return true;
  }

  /**
   * @brief 現在速度を取得する。
   * @param value 取得した速度の格納先
   * @return 成功した場合true
   */
  bool get_velocity(int16_t &value) {
    uint8_t buf[2];
    if (!read_data(0x3A, buf, sizeof(buf))) {
      return false;
    }
    std::memcpy(&value, buf, sizeof(value));
    if ((value & 0x8000) != 0) {
      value = -(value & 0x7FFF);
    }
    return true;
  }

private:
  static constexpr uint32_t TIMEOUT_MS = 3;

  /// 1バイト送信し、echoで返ってきた1バイトを読み取って返す(チェックサム計算用)。
  uint8_t write_byte(uint8_t data) {
    uart_.write(&data, 1, TIMEOUT_MS);
    uint8_t echo = 0;
    uart_.read(&echo, 1, TIMEOUT_MS);
    return echo;
  }

  /// 応答パケットを受信し、ヘッダ・チェックサムを検証する。
  bool check_reply(uint8_t *data, size_t size) {
    uint8_t header[4];
    if (!uart_.read(header, sizeof(header), TIMEOUT_MS)) {
      return false;
    }
    if (header[0] != 0xFF || header[1] != 0xFF || header[2] != id_ ||
        header[3] != size + 2) {
      return false;
    }

    uint8_t checksum = id_ + size + 2;

    uint8_t error;
    if (!uart_.read(&error, 1, TIMEOUT_MS)) {
      return false;
    }
    error_ = error;
    checksum += error;

    if (size > 0) {
      if (!uart_.read(data, size, TIMEOUT_MS)) {
        return false;
      }
      for (size_t i = 0; i < size; ++i) {
        checksum += data[i];
      }
    }

    uint8_t recv_checksum;
    if (!uart_.read(&recv_checksum, 1, TIMEOUT_MS)) {
      return false;
    }
    return recv_checksum == static_cast<uint8_t>(~checksum);
  }

  uint8_t id_;
  uint8_t error_ = 0;
  halx::driver::UARTBase &uart_;
};
