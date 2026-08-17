#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <tuple>
#include <utility>

#include <halx/core.hpp>
#include <halx/peripheral.hpp>

/// BNO055のレジスタアドレス。Page 0とPage 1でアドレスが重複する。
enum class BNO055Register : uint8_t {
  // Page 0
  CHIP_ID = 0x00,
  ACC_ID = 0x01,
  MAG_ID = 0x02,
  GYR_ID = 0x03,
  SW_REV_ID_LSB = 0x04,
  SW_REV_ID_MSB = 0x05,
  BL_REV_ID = 0x06,
  PAGE_ID = 0x07,
  ACC_DATA_X_LSB = 0x08,
  ACC_DATA_X_MSB = 0x09,
  ACC_DATA_Y_LSB = 0x0A,
  ACC_DATA_Y_MSB = 0x0B,
  ACC_DATA_Z_LSB = 0x0C,
  ACC_DATA_Z_MSB = 0x0D,
  MAG_DATA_X_LSB = 0x0E,
  MAG_DATA_X_MSB = 0x0F,
  MAG_DATA_Y_LSB = 0x10,
  MAG_DATA_Y_MSB = 0x11,
  MAG_DATA_Z_LSB = 0x12,
  MAG_DATA_Z_MSB = 0x13,
  GYR_DATA_X_LSB = 0x14,
  GYR_DATA_X_MSB = 0x15,
  GYR_DATA_Y_LSB = 0x16,
  GYR_DATA_Y_MSB = 0x17,
  GYR_DATA_Z_LSB = 0x18,
  GYR_DATA_Z_MSB = 0x19,
  EUL_DATA_X_LSB = 0x1A,
  EUL_DATA_X_MSB = 0x1B,
  EUL_DATA_Y_LSB = 0x1C,
  EUL_DATA_Y_MSB = 0x1D,
  EUL_DATA_Z_LSB = 0x1E,
  EUL_DATA_Z_MSB = 0x1F,
  QUA_DATA_W_LSB = 0x20,
  QUA_DATA_W_MSB = 0x21,
  QUA_DATA_X_LSB = 0x22,
  QUA_DATA_X_MSB = 0x23,
  QUA_DATA_Y_LSB = 0x24,
  QUA_DATA_Y_MSB = 0x25,
  QUA_DATA_Z_LSB = 0x26,
  QUA_DATA_Z_MSB = 0x27,
  LIA_DATA_X_LSB = 0x28,
  LIA_DATA_X_MSB = 0x29,
  LIA_DATA_Y_LSB = 0x2A,
  LIA_DATA_Y_MSB = 0x2B,
  LIA_DATA_Z_LSB = 0x2C,
  LIA_DATA_Z_MSB = 0x2D,
  GRV_DATA_X_LSB = 0x2E,
  GRV_DATA_X_MSB = 0x2F,
  GRV_DATA_Y_LSB = 0x30,
  GRV_DATA_Y_MSB = 0x31,
  GRV_DATA_Z_LSB = 0x32,
  GRV_DATA_Z_MSB = 0x33,
  TEMP = 0x34,
  CALIB_STAT = 0x35,
  ST_RESULT = 0x36,
  INT_STA = 0x37,
  SYS_CLK_STATUS = 0x38,
  SYS_STATUS = 0x39,
  SYS_ERR = 0x3A,
  UNIT_SEL = 0x3B,
  OPR_MODE = 0x3D,
  PWR_MODE = 0x3E,
  SYS_TRIGGER = 0x3F,
  TEMP_SOURCE = 0x40,
  AXIS_MAP_CONFIG = 0x41,
  AXIS_MAP_SIGN = 0x42,
  ACC_OFFSET_X_LSB = 0x55,
  ACC_OFFSET_X_MSB = 0x56,
  ACC_OFFSET_Y_LSB = 0x57,
  ACC_OFFSET_Y_MSB = 0x58,
  ACC_OFFSET_Z_LSB = 0x59,
  ACC_OFFSET_Z_MSB = 0x5A,
  MAG_OFFSET_X_LSB = 0x5B,
  MAG_OFFSET_X_MSB = 0x5C,
  MAG_OFFSET_Y_LSB = 0x5D,
  MAG_OFFSET_Y_MSB = 0x5E,
  MAG_OFFSET_Z_LSB = 0x5F,
  MAG_OFFSET_Z_MSB = 0x60,
  GYR_OFFSET_X_LSB = 0x61,
  GYR_OFFSET_X_MSB = 0x62,
  GYR_OFFSET_Y_LSB = 0x63,
  GYR_OFFSET_Y_MSB = 0x64,
  GYR_OFFSET_Z_LSB = 0x65,
  GYR_OFFSET_Z_MSB = 0x66,
  ACC_RADIUS_LSB = 0x67,
  ACC_RADIUS_MSB = 0x68,
  MAG_RADIUS_LSB = 0x69,
  MAG_RADIUS_MSB = 0x6A,

  // Page 1
  ACC_CONFIG = 0x08,
  MAG_CONFIG = 0x09,
  GYR_CONFIG_0 = 0x0A,
  GYR_CONFIG_1 = 0x0B,
  ACC_SLEEP_CONFIG = 0x0C,
  GYR_SLEEP_CONFIG = 0x0D,
  INT_MSK = 0x0F,
  INT_EN = 0x10,
  ACC_AM_THRES = 0x11,
  ACC_INT_SETTINGS = 0x12,
  ACC_HG_DURATION = 0x13,
  ACC_HG_THRES = 0x14,
  ACC_NM_THRES = 0x15,
  ACC_NM_SET = 0x16,
  GYR_INT_SETTING = 0x17,
  GYR_HR_X_SET = 0x18,
  GYR_DUR_X = 0x19,
  GYR_HR_Y_SET = 0x1A,
  GYR_DUR_Y = 0x1B,
  GYR_HR_Z_SET = 0x1C,
  GYR_DUR_Z = 0x1D,
  GYR_AM_THRES = 0x1E,
  GYR_AM_SET = 0x1F
};

/**
 * @brief Bosch BNO055(9軸センサフュージョンIMU)のI2Cドライバ。
 * @tparam Handle 接続されたI2Cのハンドル
 *
 * @code{.cpp}
 * #include "bno055.hpp"
 *
 * extern I2C_HandleTypeDef hi2c1;
 * BNO055<&hi2c1> imu;
 *
 * extern "C" void app_main() {
 *   imu.start();
 *
 *   if (auto euler = imu.get_euler()) {
 *     auto [yaw, roll, pitch] = *euler; // オイラー角[rad]
 *   }
 * }
 * @endcode
 */
template <I2C_HandleTypeDef *Handle> class BNO055 {
public:
  /**
   * @brief コンストラクタ。
   * @param address I2Cスレーブアドレス(8bit形式、デフォルトは`DEFAULT_ADDRESS`)
   */
  BNO055(uint16_t address = 0x28 << 1) : address_{address} {}

  /**
   * @brief
   * センサをリセットし、起動を待ってからIMUフュージョンモードで動作を開始する。
   *
   * 設定が完了するまで戻らない。
   */
  void start() {
    while (true) {
      if (!read_register<1>(BNO055Register::CHIP_ID)) {
        continue;
      }
      std::array<uint8_t, 1> data{0x20};
      write_register(BNO055Register::SYS_TRIGGER, data);
      break;
    }

    halx::core::delay(700);

    while (true) {
      std::array<uint8_t, 1> data{0x00};
      if (!write_register(BNO055Register::OPR_MODE, data)) {
        continue;
      }
      data[0] = 0x04;
      if (!write_register(BNO055Register::UNIT_SEL, data)) {
        continue;
      }
      data[0] = 0x08;
      if (!write_register(BNO055Register::OPR_MODE, data)) {
        continue;
      }
      break;
    }
  }

  /**
   * @brief オイラー角を取得する。
   * @return 成功した場合 (ヨー角(Heading), ロール角, ピッチ角)
   * [rad]、失敗した場合 std::nullopt
   */
  std::optional<std::tuple<float, float, float>> get_euler() {
    auto res = read_register<6>(BNO055Register::EUL_DATA_X_LSB);
    if (!res) {
      return std::nullopt;
    }
    float yaw = static_cast<int16_t>(((*res)[1] << 8) | (*res)[0]) / 900.0f;
    float roll = static_cast<int16_t>(((*res)[3] << 8) | (*res)[2]) / 900.0f;
    float pitch = static_cast<int16_t>(((*res)[5] << 8) | (*res)[4]) / 900.0f;
    return {{yaw, roll, pitch}};
  }

  /**
   * @brief クォータニオンを取得する。
   * @return 成功した場合 (w, x, y, z)、失敗した場合 std::nullopt
   */
  std::optional<std::tuple<float, float, float, float>> get_quaternion() {
    auto res = read_register<8>(BNO055Register::QUA_DATA_W_LSB);
    if (!res) {
      return std::nullopt;
    }
    float w = static_cast<int16_t>(((*res)[1] << 8) | (*res)[0]) / 16384.0f;
    float x = static_cast<int16_t>(((*res)[3] << 8) | (*res)[2]) / 16384.0f;
    float y = static_cast<int16_t>(((*res)[5] << 8) | (*res)[4]) / 16384.0f;
    float z = static_cast<int16_t>(((*res)[7] << 8) | (*res)[6]) / 16384.0f;
    return {{w, x, y, z}};
  }

  /**
   * @brief レジスタに書き込む。
   * @tparam N 書き込むバイト数
   * @param address 書き込み先レジスタ
   * @param data 書き込むデータ
   * @return 成功した場合true
   */
  template <size_t N>
  bool write_register(BNO055Register address,
                      const std::array<uint8_t, N> &data) {
    return halx::peripheral::ST_I2C<Handle>::mem_write(
        address_, std::to_underlying(address), 1, data.data(), data.size(),
        TIMEOUT);
  }

  /**
   * @brief レジスタから読み取る。
   * @tparam N 読み取るバイト数
   * @param address 読み取り元レジスタ
   * @return 成功した場合読み取ったデータ、失敗した場合`std::nullopt`
   */
  template <size_t N>
  std::optional<std::array<uint8_t, N>> read_register(BNO055Register address) {
    std::array<uint8_t, N> data;
    if (!halx::peripheral::ST_I2C<Handle>::mem_read(
            address_, std::to_underlying(address), 1, data.data(), data.size(),
            TIMEOUT)) {
      return std::nullopt;
    }
    return data;
  }

private:
  static constexpr uint32_t TIMEOUT = 5;

  uint16_t address_;
};
