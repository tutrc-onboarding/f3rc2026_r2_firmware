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
#include "feetech_position_control.hpp"
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
    .output_upper_limit = 1.0f,
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
UART_DMA<&huart4> uart4(uart4_tx_buf, sizeof(uart4_tx_buf), uart4_rx_buf, sizeof(uart4_rx_buf));
uint8_t uart5_tx_buf[512];
uint8_t uart5_rx_buf[512];
UART_DMA<&huart5> uart5(uart5_tx_buf, sizeof(uart5_tx_buf), uart5_rx_buf, sizeof(uart5_rx_buf));

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

constexpr float BLOCK_HOLDER_OPEN_POSITION = 100.0f / 4096.0f;
constexpr float BLOCK_HOLDER_CLOSED_POSITION = 2000.0f / 4096.0f;
constexpr float WATERING_CAN_RELEASE_POSITION = 0.0f / 4096.0f;
constexpr float WATERING_CAN_COLLECT_POSITION = 2000.0f / 4096.0f;

// FeetechPositionControl block_holder_servo(uart5, 3, BLOCK_HOLDER_OPEN_POSITION, 100, 2000);
// FeetechPositionControl watering_can_servo(uart5, 4, WATERING_CAN_RELEASE_POSITION, 100, 2000);

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

constexpr float SEQUENCE_POSITION_TOLERANCE = 0.05f; // [m]　許容誤差
constexpr uint32_t WATERING_START_TICKS = 500; // [1/100秒]倉庫Bから白ブロックを運んでから何秒待って水やりを開始するか
uint32_t competition_ticks = 0;                // 競技時間を計測
uint32_t waiting_ticks = 0;                    // どんくらい待ってるか
bool competition_running = false;              // 計測のトリガー的な

// R2スタートゾーンの中心を原点、右を+x、上を+y
constexpr Pose R2_START_POSE{0.0f, 0.0f, 0.0f};
constexpr Pose WAREHOUSE_C_POSE{-1.60f, 0.075f, -0.5f * std::numbers::pi};
constexpr Pose WAREHOUSE_B_POSE{-1.60f, 0.90f, -0.5f * std::numbers::pi};
constexpr Pose WAREHOUSE_A_POSE{-1.60f, 1.725f, 0.0f};
constexpr Pose GARDEN_BLACK_BLOCK_POSE{1.65f, 0.30f, 0.5f * std::numbers::pi};
constexpr Pose GARDEN_WHITE_BLOCK_POSE{1.65f, 0.90f, 0.0f};
constexpr Pose GARDEN_WATERING_POSE{1.65f, 0.90f, 0.0f};

// コントロールモード一覧
enum class AutoControlMode {
  EMERGENCY_STOP,
  MANUAL,
  START_TO_C,
  GET_BLOCK_AND_WATERING_CAN,
  C_TO_GARDEN,
  PUT_BLACK_BLOCK,
  /// 自動機がBの白ブロックを２個回収するかも、ということで書いておいた　使わないかも
  GARDEN_TO_B,
  GET_WHITE_BLOCK,
  B_TO_GARDEN,
  PUT_WHITE_BLOCK,
  ///
  WAIT_FOR_WATERING,
  GARDEN_TO_C_WARTERING,
  C_TO_GARDEN_WARTERING,
  GARDEN_TO_A_WARTERING,
  A_TO_GARDEN_WARTERING,

};

Pose robot_pose = R2_START_POSE;
AutoControlMode auto_control_mode = AutoControlMode::EMERGENCY_STOP;

void timer_callback(void *);
void update_localization();
Velocity calculate_velocity(const Pose &now_pose, const Pose &target_pose);
void drive_wheels(const Velocity &cmd_vel);
void stop_drive_wheels();
void set_auto_control_mode(AutoControlMode mode);
void move_to_pose(const Pose &target_pose, AutoControlMode next_mode);
void move_servo(FeetechPositionControl &servo, float target_position, AutoControlMode next_mode);
void collect_block_and_watering_can();

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

  // block_holder_servo.start();
  // watering_can_servo.start();

  ST_TIM<&htim6>::register_period_elapsed_callback(timer_callback, nullptr);
  ST_TIM<&htim6>::start_base_it();

  while (true) {
    if (auto euler = imu.get_euler()) {
      imu_yaw = std::get<0>(*euler);
    }

    // block_holder_servo.update();
    // watering_can_servo.update();

    // printf("x: %f, y: %f, yaw: %f, block_pos: %f, watering_pos: %f\n\r", debug_pose_x.load(), debug_pose_y.load(),
    //        debug_pose_yaw.load(), block_holder_servo.get_position(), watering_can_servo.get_position());

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

  if (ps3.get_key_down(PS3Key::SELECT)) {
    competition_running = false;
    set_auto_control_mode(AutoControlMode::EMERGENCY_STOP);
  }

  switch (auto_control_mode) {
  case AutoControlMode::EMERGENCY_STOP:
    stop_drive_wheels();
    if (ps3.get_key(PS3Key::L1) && ps3.get_key(PS3Key::R1)) {
      set_auto_control_mode(AutoControlMode::MANUAL);
    }
    break;

  case AutoControlMode::MANUAL: {
    if (ps3.get_key_down(PS3Key::START)) {
      robot_pose = R2_START_POSE;
      competition_ticks = 0;
      competition_running = true;
      // block_holder_servo.set_position(BLOCK_HOLDER_OPEN_POSITION);
      // watering_can_servo.set_position(WATERING_CAN_RELEASE_POSITION);
      stop_drive_wheels();
      set_auto_control_mode(AutoControlMode::START_TO_C);
      break;
    }
    // メモ　デバッグするときは下のコメントアウトを外してset_auto_control_modeをコメントアウトする
    //  if (ps3.get_key_down(PS3Key::LEFT)) {
    //    block_holder_servo.set_position(BLOCK_HOLDER_OPEN_POSITION);
    //  }
    //  if (ps3.get_key_down(PS3Key::RIGHT)) {
    //    block_holder_servo.set_position(BLOCK_HOLDER_CLOSED_POSITION);
    //  }
    //  if (ps3.get_key_down(PS3Key::UP)) {
    //    watering_can_servo.set_position(WATERING_CAN_RELEASE_POSITION);
    //  }
    //  if (ps3.get_key_down(PS3Key::DOWN)) {
    //    watering_can_servo.set_position(WATERING_CAN_COLLECT_POSITION);
    //  }

    Velocity velocity;
    velocity.x = 0.5f * ps3.get_axis(PS3Axis::LEFT_X);
    velocity.y = 0.5f * ps3.get_axis(PS3Axis::LEFT_Y);
    velocity.yaw = -(std::numbers::pi / 2.0f) * ps3.get_axis(PS3Axis::RIGHT_X); // 反時計回りに正となるように符号を反転
    drive_wheels(velocity);
    break;
  }
  // move_to_pose(行く場所, 次の動作)
  // move_servo(動かすサーボ, set_position, 次の動作)
  case AutoControlMode::START_TO_C:
    move_to_pose(WAREHOUSE_C_POSE, AutoControlMode::GET_BLOCK_AND_WATERING_CAN);
    break;

  case AutoControlMode::GET_BLOCK_AND_WATERING_CAN:
    collect_block_and_watering_can();
    // 場所が無かったからここにCに行く動作を書いた
    set_auto_control_mode(AutoControlMode::C_TO_GARDEN);
    break;

  case AutoControlMode::C_TO_GARDEN:
    move_to_pose(GARDEN_BLACK_BLOCK_POSE, AutoControlMode::PUT_BLACK_BLOCK);
    break;

  case AutoControlMode::PUT_BLACK_BLOCK:

    // move_servo(block_holder_servo, BLOCK_HOLDER_OPEN_POSITION, AutoControlMode::GARDEN_TO_B);
    set_auto_control_mode(AutoControlMode::GARDEN_TO_B);
    break;

  case AutoControlMode::GARDEN_TO_B:
    move_to_pose(WAREHOUSE_B_POSE, AutoControlMode::GET_WHITE_BLOCK);
    break;

  case AutoControlMode::GET_WHITE_BLOCK:
    // move_servo(block_holder_servo, BLOCK_HOLDER_CLOSED_POSITION, AutoControlMode::B_TO_GARDEN);
    break;

  case AutoControlMode::B_TO_GARDEN:
    move_to_pose(GARDEN_WHITE_BLOCK_POSE, AutoControlMode::PUT_WHITE_BLOCK);
    break;

  case AutoControlMode::PUT_WHITE_BLOCK:
    // move_servo(block_holder_servo, BLOCK_HOLDER_OPEN_POSITION, AutoControlMode::WAIT_FOR_WATERING);
    break;

  case AutoControlMode::WAIT_FOR_WATERING:
    waiting_ticks = waiting_ticks + 1;
    // 邪魔だったら待機場所を設定してもいいかも
    stop_drive_wheels();
    if (waiting_ticks >= WATERING_START_TICKS) {
      set_auto_control_mode(AutoControlMode::GARDEN_TO_C_WARTERING);
    }
    break;

  case AutoControlMode::GARDEN_TO_C_WARTERING:
    move_to_pose(WAREHOUSE_C_POSE, AutoControlMode::C_TO_GARDEN_WARTERING);
    break;

  case AutoControlMode::C_TO_GARDEN_WARTERING:
    move_to_pose(GARDEN_WATERING_POSE, AutoControlMode::GARDEN_TO_A_WARTERING);
    break;

  case AutoControlMode::GARDEN_TO_A_WARTERING:
    move_to_pose(WAREHOUSE_A_POSE, AutoControlMode::A_TO_GARDEN_WARTERING);
    break;

  case AutoControlMode::A_TO_GARDEN_WARTERING:
    move_to_pose(GARDEN_WATERING_POSE, AutoControlMode::GARDEN_TO_C_WARTERING);
    break;
  }

  if (competition_running) {
    ++competition_ticks;
  }
  debug_pose_x = robot_pose.x;
  debug_pose_y = robot_pose.y;
  debug_pose_yaw = robot_pose.yaw;
}

void set_auto_control_mode(AutoControlMode mode) { auto_control_mode = mode; }

void move_to_pose(const Pose &target_pose, AutoControlMode next_mode) {
  const float delta_x = target_pose.x - robot_pose.x;
  const float delta_y = target_pose.y - robot_pose.y;
  constexpr float POSITION_TOLERANCE_SQUARED = SEQUENCE_POSITION_TOLERANCE * SEQUENCE_POSITION_TOLERANCE;

  if (delta_x * delta_x + delta_y * delta_y <= POSITION_TOLERANCE_SQUARED) {
    stop_drive_wheels();
    set_auto_control_mode(next_mode);
    return;
  }

  drive_wheels(calculate_velocity(robot_pose, target_pose));
}

void move_servo(FeetechPositionControl &servo, float target_position, AutoControlMode next_mode) {
  stop_drive_wheels();
  // servo.set_position(target_position);
  set_auto_control_mode(next_mode);
}

void collect_block_and_watering_can() { // ブロックとじょうろが同時に取れる前提で書いた
  stop_drive_wheels();
  // block_holder_servo.set_position(BLOCK_HOLDER_CLOSED_POSITION);
  // watering_can_servo.set_position(WATERING_CAN_COLLECT_POSITION);
}

void update_localization() {
  // 角度差分をとる
  float raw_yaw = imu_yaw;
  static float pre_raw_yaw = raw_yaw;
  float delta_yaw = -(raw_yaw - pre_raw_yaw); // 反時計回りに正となるように符号を反転
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
  const float rev_to_distance = 2.0f * std::numbers::pi * ODOMETRY_WHEEL_RADIUS; // 1回転あたりの移動距離[m]
  float delta_x = (x_encoder.get_position() - pre_x_position) * rev_to_distance;
  float delta_y =
      -(y_encoder.get_position() - pre_y_position) * rev_to_distance; // y軸エンコーダが逆向きに回転するため符号を反転
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

Velocity calculate_velocity(const Pose &now_pose, const Pose &target_pose) {
  static PIDController p2p_x_pid(P2P_X_PID_PARAMS, CONTROL_DT);
  static PIDController p2p_y_pid(P2P_Y_PID_PARAMS, CONTROL_DT);
  static PIDController p2p_yaw_pid(P2P_YAW_PID_PARAMS, CONTROL_DT);

  Velocity world_velocity;
  world_velocity.x = p2p_x_pid.solve(target_pose.x - now_pose.x);
  world_velocity.y = p2p_y_pid.solve(target_pose.y - now_pose.y);
  world_velocity.yaw = p2p_yaw_pid.solve(target_pose.yaw - now_pose.yaw);

  Velocity robot_velocity;
  robot_velocity.x = world_velocity.x * std::cos(now_pose.yaw) + world_velocity.y * std::sin(now_pose.yaw);
  robot_velocity.y = world_velocity.y * std::cos(now_pose.yaw) - world_velocity.x * std::sin(now_pose.yaw);
  robot_velocity.yaw = world_velocity.yaw;

  return robot_velocity;
}

void drive_wheels(const Velocity &velocity) {
  static PIDController motor1_pid(DRIVE_WHEEL_PID_PARAMS, CONTROL_DT);
  static PIDController motor2_pid(DRIVE_WHEEL_PID_PARAMS, CONTROL_DT);
  static PIDController motor3_pid(DRIVE_WHEEL_PID_PARAMS, CONTROL_DT);

  constexpr float VEL2RPS = 1.0f / (2.0f * std::numbers::pi * DRIVE_WHEEL_RADIUS);

  float motor1_target_rps = (-velocity.x * std::sin(DRIVE_WHEEL_THETA_1) + velocity.y * std::cos(DRIVE_WHEEL_THETA_1) +
                             ROBOT_RADIUS * velocity.yaw) *
                            VEL2RPS;
  float motor2_target_rps = (-velocity.x * std::sin(DRIVE_WHEEL_THETA_2) + velocity.y * std::cos(DRIVE_WHEEL_THETA_2) +
                             ROBOT_RADIUS * velocity.yaw) *
                            VEL2RPS;
  float motor3_target_rps = (-velocity.x * std::sin(DRIVE_WHEEL_THETA_3) + velocity.y * std::cos(DRIVE_WHEEL_THETA_3) +
                             ROBOT_RADIUS * velocity.yaw) *
                            VEL2RPS;

  float motor1_output = motor1_pid.solve(motor1_target_rps - motor1_encoder.get_rps());
  float motor2_output = motor2_pid.solve(motor2_target_rps - motor2_encoder.get_rps());
  float motor3_output = motor3_pid.solve(motor3_target_rps - (-motor3_encoder.get_rps()));

  motor1.set_output(motor1_output);
  motor2.set_output(motor2_output);
  motor3.set_output(motor3_output);
}

void stop_drive_wheels() {
  motor1.set_output(0.0f);
  motor2.set_output(0.0f);
  motor3.set_output(0.0f);
}