#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <numbers>

#include <halx/core.hpp>
#include <halx/driver/gpio.hpp>
#include <halx/driver/uart_dma.hpp>
#include <halx/driver/uart_it.hpp>
#include <halx/peripheral.hpp>

#include "bno055.hpp"
#include "encoder.hpp"
#include "feetech_servo.hpp"
#include "main.h"
#include "motor.hpp"
#include "pid_controller.hpp"
#include "ps3.hpp"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim20;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef hlpuart1;
extern I2C_HandleTypeDef hi2c3;

using halx::driver::GPIO;
using halx::driver::UART_DMA;
using halx::driver::UART_IT;
using halx::peripheral::ST_TIM;

constexpr float CONTROL_DT = 0.01f;

constexpr float ROBOT_RADIUS = 0.177f;
constexpr float DRIVE_WHEEL_RADIUS = 0.05f;
constexpr float ODOMETRY_WHEEL_RADIUS = 0.03f;

constexpr float DRIVE_WHEEL_THETA_1 = std::numbers::pi / 6.0f;
constexpr float DRIVE_WHEEL_THETA_2 = std::numbers::pi * 5.0f / 6.0f;
constexpr float DRIVE_WHEEL_THETA_3 = std::numbers::pi * 3.0f / 2.0f;

constexpr PIDParameters DRIVE_WHEEL_PID_PARAMS{
    .kp = 0.01f,
    .ki = 0.7f,
    .kd = 0.0f,
    .output_upper_limit = 0.7f,
    .integral_upper_limit = 1.0f,
};

constexpr PIDParameters P2P_X_PID_PARAMS{
    .kp = 1.0f,
    .output_upper_limit = 0.5f,
};
constexpr PIDParameters P2P_Y_PID_PARAMS{
    .kp = 1.0f,
    .output_upper_limit = 0.5f,
};
constexpr PIDParameters P2P_YAW_PID_PARAMS{
    .kp = 1.0f,
    .output_upper_limit = std::numbers::pi / 2.0f,
};


UART_IT<&hlpuart1> lpuart1;
uint8_t uart4_tx_buf[512];
uint8_t uart4_rx_buf[512];
UART_DMA<&huart4> uart4(uart4_tx_buf, sizeof(uart4_tx_buf), uart4_rx_buf,
                        sizeof(uart4_rx_buf));
uint8_t uart5_tx_buf[512];
uint8_t uart5_rx_buf[512];
UART_DMA<&huart5> uart5(uart5_tx_buf, sizeof(uart5_tx_buf), uart5_rx_buf,
                        sizeof(uart5_rx_buf));

GPIO motor1_pin(Motor8_GPIO_Port, Motor8_Pin);
GPIO motor2_pin(Motor5_GPIO_Port, Motor5_Pin);
GPIO motor3_pin(Motor4_GPIO_Port, Motor4_Pin);

Encoder<&htim8> motor1_encoder(2048, 2.0f, CONTROL_DT);
Encoder<&htim5> motor2_encoder(2048, 2.0f, CONTROL_DT);
Encoder<&htim4> motor3_encoder(2048, 2.0f, CONTROL_DT);
Encoder<&htim1> x_encoder(2048, 1.0f, CONTROL_DT);
Encoder<&htim3> y_encoder(2048, 1.0f, CONTROL_DT);

Motor<&htim15> motor1(TIM_CHANNEL_1, motor1_pin);
Motor<&htim20> motor2(TIM_CHANNEL_2, motor2_pin);
Motor<&htim20> motor3(TIM_CHANNEL_1, motor3_pin);

PS3 ps3(uart4);
BNO055<&hi2c3> imu;
FeetechServo servo(0x1, uart5);

std::atomic<float> imu_yaw = 0.0f;

std::atomic<float> debug_pose_x = 0.0f;
std::atomic<float> debug_pose_y = 0.0f;
std::atomic<float> debug_pose_yaw = 0.0f;

struct Velocity {
  float x;   // [m/s]
  float y;   // [m/s]
  float yaw; // [rad/s]
};

struct Pose {
  float x;   // [m]
  float y;   // [m]
  float yaw; // [rad]
};

constexpr float SEQUENCE_POSITION_TOLERANCE = 0.05f; // [m]
// 起動地点と起動時の向きを (0 m, 0 m, 0 rad) とする絶対目標。
constexpr Pose HOME_POSE{0.0f, 0.0f, 0.0f};

// 目標ポイント一覧
constexpr std::array<Pose, 3> SEQUENCE_TARGET_POSES{{
  // {x, y, w}
    {0.0f, 1.0f, 0.0f},
    {1.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
}};

// コントロールモード一覧
enum class AutoControlMode {
  IDLE,
  POSE_SEQUENCE,
  RETURN_HOME,
};

Pose robot_pose;
AutoControlMode auto_control_mode = AutoControlMode::IDLE;
int sequence_target_index = 0;

void timer_callback(void *);
void update_localization();
Velocity calc_p2p_velocity(const Pose &now_pose, const Pose &target_pose);
void start_pose_sequence();
void start_return_home();
bool update_auto_control(const Pose &now_pose, Velocity &cmd_vel);
void set_pose_target_velocity(const Pose &now_pose, const Pose &target_pose,
                              Velocity &cmd_vel);
void drive_wheels(const Velocity &cmd_vel);

extern "C" void app_main() {
  halx::driver::enable_stdout(lpuart1);

  uart4.start();
  uart5.start();
  lpuart1.start();

  motor1_encoder.start();
  motor2_encoder.start();
  motor3_encoder.start();
  x_encoder.start();
  y_encoder.start();

  motor1.start();
  motor2.start();
  motor3.start();

  imu.start();

  start_pose_sequence();

  ST_TIM<&htim6>::register_period_elapsed_callback(timer_callback, nullptr);
  ST_TIM<&htim6>::start_base_it();

  while (true) {
    if (auto euler = imu.get_euler()) {
      imu_yaw = std::get<0>(*euler);
    }

    printf("Now X: %f, Now Y: %f, Now Yaw: %f\n\r", debug_pose_x.load(),
           debug_pose_y.load(), debug_pose_yaw.load());

    halx::core::delay(10);
  }
}

void timer_callback(void *) {
  motor1_encoder.update();
  motor2_encoder.update();
  motor3_encoder.update();
  x_encoder.update();
  y_encoder.update();
  ps3.update();

  update_localization();

  Velocity cmd_vel{
      0.5f * ps3.get_axis(PS3Axis::LEFT_X),
      0.5f * ps3.get_axis(PS3Axis::LEFT_Y),
      -(std::numbers::pi / 2.0f) *
          ps3.get_axis(
              PS3Axis::RIGHT_X), // 反時計回りに正となるように符号を反転
  };

  if (ps3.get_key_down(PS3Key::CROSS)) {
    start_return_home();
  }

  update_auto_control(robot_pose, cmd_vel);

  drive_wheels(cmd_vel);

  debug_pose_x = robot_pose.x;
  debug_pose_y = robot_pose.y;
  debug_pose_yaw = robot_pose.yaw;
}

void update_localization() {
  // 角度差分をとる
  float raw_yaw = imu_yaw;
  static float pre_raw_yaw = raw_yaw;
  float delta_yaw =
      -(raw_yaw - pre_raw_yaw); // 反時計回りに正となるように符号を反転
  if (delta_yaw > std::numbers::pi) {
    delta_yaw -= 2.0f * std::numbers::pi;
  } else if (delta_yaw < -std::numbers::pi) {
    delta_yaw += 2.0f * std::numbers::pi;
  }
  pre_raw_yaw = raw_yaw;
  robot_pose.yaw += delta_yaw; // 角度の累積

  // オドメータの更新
  static float pre_x_position = x_encoder.get_position();
  static float pre_y_position = y_encoder.get_position();
  const float rev_to_distance =
      2.0f * std::numbers::pi *
      ODOMETRY_WHEEL_RADIUS; // 1回転あたりの移動距離[m]
  float delta_x = (x_encoder.get_position() - pre_x_position) * rev_to_distance;
  float delta_y =
      -(y_encoder.get_position() - pre_y_position) *
      rev_to_distance; // y軸エンコーダが逆向きに回転するため符号を反転
  pre_x_position = x_encoder.get_position();
  pre_y_position = y_encoder.get_position();

  // ロボット座標系からワールド座標系に変換
  float cos_yaw = std::cos(robot_pose.yaw);
  float sin_yaw = std::sin(robot_pose.yaw);

  // ワールド座標系での変位を計算
  float world_delta_x = delta_x * cos_yaw - delta_y * sin_yaw;
  float world_delta_y = delta_x * sin_yaw + delta_y * cos_yaw;
  robot_pose.x += world_delta_x;
  robot_pose.y += world_delta_y;
}

Velocity calc_p2p_velocity(const Pose &now_pose, const Pose &target_pose) {
  static PIDController p2p_x_pid(P2P_X_PID_PARAMS, CONTROL_DT);
  static PIDController p2p_y_pid(P2P_Y_PID_PARAMS, CONTROL_DT);
  static PIDController p2p_yaw_pid(P2P_YAW_PID_PARAMS, CONTROL_DT);

  // 目標位置までの差分を計算
  float delta_x = target_pose.x - now_pose.x;
  float delta_y = target_pose.y - now_pose.y;
  float delta_yaw = target_pose.yaw - now_pose.yaw;

  return {
      p2p_x_pid.solve(delta_x),
      p2p_y_pid.solve(delta_y),
      p2p_yaw_pid.solve(delta_yaw),
  };
}

void start_pose_sequence() {
  sequence_target_index = 0;
  auto_control_mode = AutoControlMode::POSE_SEQUENCE;
}

void start_return_home() {
  auto_control_mode = AutoControlMode::RETURN_HOME;
}

bool update_auto_control(const Pose &now_pose, Velocity &cmd_vel) {
  if (auto_control_mode == AutoControlMode::IDLE) {
    return false;
  }

  if (auto_control_mode == AutoControlMode::RETURN_HOME) {
    set_pose_target_velocity(now_pose, HOME_POSE, cmd_vel);
    return true;
  }

  const Pose &target_pose = SEQUENCE_TARGET_POSES[sequence_target_index]; //目標ポイントを更新
  const float delta_x = target_pose.x - now_pose.x;
  const float delta_y = target_pose.y - now_pose.y;
  const float position_error_squared = delta_x * delta_x + delta_y * delta_y; //目標ポイントとの差分を計算
  constexpr float position_tolerance_squared =
      SEQUENCE_POSITION_TOLERANCE * SEQUENCE_POSITION_TOLERANCE;

  if (position_error_squared <= position_tolerance_squared) {
    ++sequence_target_index;
    if (sequence_target_index >= SEQUENCE_TARGET_POSES.size()) { //シーケンス達成回数が設定した要素数を超えたら開始地点に戻る
      auto_control_mode = AutoControlMode::IDLE;
      cmd_vel = {0.0f, 0.0f, 0.0f};
      return true;
    }
  }

  set_pose_target_velocity(now_pose,
                           SEQUENCE_TARGET_POSES[sequence_target_index], cmd_vel);
  return true;
}

void set_pose_target_velocity(const Pose &now_pose, const Pose &target_pose,
                              Velocity &cmd_vel) {
  const Velocity world_velocity = calc_p2p_velocity(now_pose, target_pose);

  const float cos_yaw = std::cos(now_pose.yaw);
  const float sin_yaw = std::sin(now_pose.yaw);
  cmd_vel.x = world_velocity.x * cos_yaw + world_velocity.y * sin_yaw;
  cmd_vel.y = world_velocity.y * cos_yaw - world_velocity.x * sin_yaw;
  cmd_vel.yaw = world_velocity.yaw;
}

void drive_wheels(const Velocity &cmd_vel) {
  static PIDController motor1_pid(DRIVE_WHEEL_PID_PARAMS, CONTROL_DT);
  static PIDController motor2_pid(DRIVE_WHEEL_PID_PARAMS, CONTROL_DT);
  static PIDController motor3_pid(DRIVE_WHEEL_PID_PARAMS, CONTROL_DT);

  float vr_vel = ROBOT_RADIUS * cmd_vel.yaw;
  float vel2rps = 1.0f / (2.0f * std::numbers::pi * DRIVE_WHEEL_RADIUS);

  float motor1_target_rps =
      (-cmd_vel.x * std::sin(DRIVE_WHEEL_THETA_1) +
       cmd_vel.y * std::cos(DRIVE_WHEEL_THETA_1) + vr_vel) *
      vel2rps;
  float motor2_target_rps =
      (-cmd_vel.x * std::sin(DRIVE_WHEEL_THETA_2) +
       cmd_vel.y * std::cos(DRIVE_WHEEL_THETA_2) + vr_vel) *
      vel2rps;
  float motor3_target_rps =
      (-cmd_vel.x * std::sin(DRIVE_WHEEL_THETA_3) +
       cmd_vel.y * std::cos(DRIVE_WHEEL_THETA_3) + vr_vel) *
      vel2rps;

  float motor1_output =
      motor1_pid.solve(motor1_target_rps - motor1_encoder.get_rps());
  float motor2_output =
      motor2_pid.solve(motor2_target_rps - motor2_encoder.get_rps());
  float motor3_output =
      motor3_pid.solve(motor3_target_rps - (-motor3_encoder.get_rps()));

  motor1.set_output(motor1_output);
  motor2.set_output(motor2_output);
  motor3.set_output(motor3_output);
}