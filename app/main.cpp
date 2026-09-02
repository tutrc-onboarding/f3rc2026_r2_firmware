#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <numbers>

#include <halx/core.hpp>
#include <halx/driver/gpio.hpp>
#include <halx/driver/uart_base.hpp>
#include <halx/driver/uart_dma.hpp>
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
using halx::peripheral::ST_TIM;

class BlockingUART final : public halx::driver::UARTBase {
public:
  bool start() override { return true; }
  bool stop() override { return true; }

  bool write(const uint8_t *data, size_t size, uint32_t timeout) override {
    return HAL_UART_Transmit(&hlpuart1, const_cast<uint8_t *>(data), static_cast<uint16_t>(size), timeout) == HAL_OK;
  }

  bool read(uint8_t *, size_t, uint32_t) override { return false; }
  void clear() override {}
  size_t available() const override { return 0; }
};

constexpr float CONTROL_DT = 0.01f;

constexpr float ROBOT_RADIUS = 0.177f;
constexpr float DRIVE_WHEEL_RADIUS = 0.05f;
constexpr float ODOMETRY_WHEEL_RADIUS = 0.03f;

constexpr float DRIVE_WHEEL_THETA_1 = std::numbers::pi / 6.0f;
constexpr float DRIVE_WHEEL_THETA_2 = std::numbers::pi * 5.0f / 6.0f;
constexpr float DRIVE_WHEEL_THETA_3 = std::numbers::pi * 3.0f / 2.0f;

constexpr PIDParameters DRIVE_WHEEL_PID_PARAMS{
    .kp = 0.03f,
    .ki = 0.7f,
    .kd = 0.0f,
    .output_upper_limit = 1.0f,
    .integral_upper_limit = 1.0f,
};

constexpr PIDParameters P2P_X_PID_PARAMS{
    .kp = 1.5f,
    .output_upper_limit = 0.5f,
};
constexpr PIDParameters P2P_Y_PID_PARAMS{
    .kp = 1.5f,
    .output_upper_limit = 0.5f,
};
constexpr PIDParameters P2P_YAW_PID_PARAMS{
    .kp = 2.0f,
    .output_upper_limit = std::numbers::pi,
};

BlockingUART debug_uart;
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

PIDController motor1_pid(DRIVE_WHEEL_PID_PARAMS, CONTROL_DT);
PIDController motor2_pid(DRIVE_WHEEL_PID_PARAMS, CONTROL_DT);
PIDController motor3_pid(DRIVE_WHEEL_PID_PARAMS, CONTROL_DT);

PS3 ps3(uart4);
BNO055<&hi2c3> imu;

constexpr int BLOCK_HOLDER_OPEN_POSITION = 2414;
constexpr int BLOCK_HOLDER_CLOSED_POSITION = 3797;
constexpr int WATERING_CAN_PULL_POSITION = 2575;
constexpr int WATERING_CAN_COLLECT_POSITION = 1898;

FeetechPositionControl block_holder_servo(uart5, 1, 3787); // 2414-3146
FeetechPositionControl watering_can_servo(uart5, 2, 3431); // 597-3051 (get:895, pull:1532)

std::atomic<float> imu_yaw = 0.0f;

std::atomic<float> debug_pose_x = 0.0f;
std::atomic<float> debug_pose_y = 0.0f;
std::atomic<float> debug_pose_yaw = 0.0f;

struct Velocity {
  float x;   // [m/s]
  float y;   // [m/s]
  float yaw; // [rad/s]
};

constexpr float MAX_IDOU_ACCEL = 1.2f; // [m/s^2]
constexpr float MAX_YAW_ACCEL = 6.0f;  // [rad/s^2]
Velocity pre_velocity{0.0f, 0.0f, 0.0f};

struct Pose {
  float x;   // [m]
  float y;   // [m]
  float yaw; // [rad]
};

constexpr float SEQUENCE_X_POSITION_TOLERANCE = 0.01f;      // [m] x軸方向の許容誤差
constexpr float SEQUENCE_Y_POSITION_TOLERANCE = 0.05f;      // [m] y軸方向の許容誤差
constexpr float SEQUENCE_YAW_TOLERANCE = 0.025f;            // [rad] 角度の許容誤差
constexpr float WAREHOUSE_ENTRY_POSITION_TOLERANCE = 0.01f; // [m] 倉庫進入時の座標調整を確実に反映するための許容誤差
constexpr float WAREHOUSE_ENTRY_MAX_SPEED = 0.15f;          // [m/s] 待機点から回収点までの最大並進速度
constexpr float WAREHOUSE_WAIT_OFFSET_X = 0.20f;            // [m] 回収点から待機点までのx方向オフセット
constexpr float BLOCK_BACK_DISTANCE = 0.35f;                // [m] ブロック配置後の後退距離
constexpr uint32_t WAIT_TICKS_MECHA = 25;                   // [1/100秒] 回収・設置後に～秒待つ
constexpr uint32_t WATERING_START_TICKS = 500; // [1/100秒]倉庫Bから白ブロックを運んでから何秒待って水やりを開始するか
uint32_t competition_ticks = 0;                // 競技時間を計測
uint32_t waiting_ticks = 0;                    // どんくらい待ってるか
uint32_t waiting_ticks_mecha = 0;
bool competition_running = false; // 計測のトリガー的な

// R2スタートゾーンの中心を原点、右を+x、上を+y
constexpr Pose R2_START_POSE{0.0f, 0.0f, -0.5f * std::numbers::pi};
constexpr Pose WAREHOUSE_C_POSE{-1.40f, 0.25f, -0.5f * std::numbers::pi};
constexpr Pose WAREHOUSE_C_WAIT_POSE{WAREHOUSE_C_POSE.x + WAREHOUSE_WAIT_OFFSET_X, WAREHOUSE_C_POSE.y,
                                     WAREHOUSE_C_POSE.yaw};
constexpr Pose WAREHOUSE_B_POSE{-1.10f, 0.95, -0.5f * std::numbers::pi};
constexpr Pose WAREHOUSE_B_WAIT_POSE{WAREHOUSE_B_POSE.x + WAREHOUSE_WAIT_OFFSET_X, WAREHOUSE_B_POSE.y,
                                     WAREHOUSE_B_POSE.yaw};
constexpr Pose WAREHOUSE_A_POSE{-1.30f, 1.725f, -0.5f * std::numbers::pi};
constexpr Pose GARDEN_BLACK_BLOCK_POSE{1.65f, 1.725f, 0.5f * std::numbers::pi};
constexpr Pose GARDEN_WHITE_BLOCK_POSE{1.65f, 0.90f, 0.5f * std::numbers::pi};
constexpr Pose GARDEN_WATERING_POSE{1.65f, 1.20f, -0.5f * std::numbers::pi};
// ↓作業後の座標
constexpr Pose WAREHOUSE_C_EXIT_POSE{-0.4f, 0.25f, WAREHOUSE_C_POSE.yaw};
constexpr Pose WAREHOUSE_C_EXIT_ROTATED_POSE{WAREHOUSE_C_EXIT_POSE.x, WAREHOUSE_C_EXIT_POSE.y, WAREHOUSE_C_POSE.yaw};
constexpr Pose GARDEN_BLACK_BLOCK_BACK_POSE{GARDEN_BLACK_BLOCK_POSE.x - BLOCK_BACK_DISTANCE, GARDEN_BLACK_BLOCK_POSE.y,
                                            GARDEN_BLACK_BLOCK_POSE.yaw};
constexpr Pose WAREHOUSE_B_EXIT_POSE{-0.4f, WAREHOUSE_B_POSE.y, WAREHOUSE_B_POSE.yaw};
constexpr Pose WAREHOUSE_B_EXIT_ROTATED_POSE{WAREHOUSE_B_EXIT_POSE.x, WAREHOUSE_B_EXIT_POSE.y,
                                             WAREHOUSE_B_EXIT_POSE.yaw};
constexpr Pose GARDEN_WHITE_BLOCK_BACK_POSE{GARDEN_WHITE_BLOCK_POSE.x - BLOCK_BACK_DISTANCE, GARDEN_WHITE_BLOCK_POSE.y,
                                            GARDEN_WHITE_BLOCK_POSE.yaw};
constexpr Pose GARDEN_WHITE_BLOCK_EXIT_POSE{GARDEN_WHITE_BLOCK_BACK_POSE.x, GARDEN_WHITE_BLOCK_BACK_POSE.y,
                                            -0.5f * std::numbers::pi};
constexpr Pose WATERING_WAREHOUSE_A{-1.3, 0.085f, -0.5f * std::numbers::pi};
constexpr Pose WATERING_WAREHOUSE_A_RELAY{-1.0, 0.085, -0.5f * std::numbers::pi};
constexpr Pose WATERING_GARDEN{1.5f, 0.905f, -0.5f * std::numbers::pi};
constexpr Pose WATERING_WAREHOUSE_C{-1.3, 1.725f, -0.5f * std::numbers::pi};
constexpr Pose WATERING_WAREHOUSE_C_RELAY{-1.0, 1.725f, -0.5f * std::numbers::pi};
// コントロールモード一覧
enum class AutoControlMode {
  EMERGENCY_STOP,
  MANUAL,
  START_TO_C,
  ENTER_WAREHOUSE_C,
  GET_BLACK_BLOCK,
  WAIT_GET_BLACK_BLOCK,
  EXIT_WAREHOUSE_C,
  ROTATE_AFTER_EXIT_WAREHOUSE_C,
  C_TO_GARDEN,
  PUT_BLACK_BLOCK,
  WAIT_PUT_BLACK_BLOCK,
  BACK_FROM_BLACK_BLOCK_GARDEN,
  /// 自動機がBの白ブロックを２個回収するかも、ということで書いておいた　使わないかも
  GARDEN_TO_B,
  ENTER_WAREHOUSE_B,
  GET_WHITE_BLOCK,
  WAIT_GET_WHITE_BLOCK,
  EXIT_WAREHOUSE_B,
  ROTATE_AFTER_EXIT_WAREHOUSE_B,
  B_TO_GARDEN,
  PUT_WHITE_BLOCK,
  WAIT_PUT_WHITE_BLOCK,
  BACK_FROM_WHITE_BLOCK_GARDEN,
  ROTATE_AFTER_WHITE_BLOCK,
  ///
  WAIT_FOR_WATERING,
  WATERING_TO_A_RELAY,
  WATERING_A_RELAY_TO_A,
  WATERING_A_TO_A_RELAY,
  WATERING_A_RELAY_TO_GARDEN,
  WATERING_GARDEN_TO_C_RELAY,
  WATERING_C_RELAY_TO_C,
  WATERING_C_TO_C_RELAY,
  WATERING_C_RELAY_TO_GARDEN,

};

Pose robot_pose = R2_START_POSE;
AutoControlMode auto_control_mode = AutoControlMode::EMERGENCY_STOP;

void timer_callback(void *);
void update_localization();
Velocity calculate_velocity(const Pose &now_pose, const Pose &target_pose);
Velocity limit_acceleration(const Velocity &target_velocity);
void drive_wheels(const Velocity &cmd_vel);
void stop_drive_wheels();
void stop_drive_wheels_for_pause();
void set_auto_control_mode(AutoControlMode mode);
void move_to_pose(const Pose &target_pose, AutoControlMode next_mode, float max_translation_speed = 0.0f,
                  float x_position_tolerance = SEQUENCE_X_POSITION_TOLERANCE,
                  float y_position_tolerance = SEQUENCE_Y_POSITION_TOLERANCE);
void move_to_position_without_rotation(const Pose &target_pose, AutoControlMode next_mode);
void rotate_without_translation(const Pose &target_pose, AutoControlMode next_mode);
void wait_for_mecha(AutoControlMode next_mode);
void move_servo(FeetechPositionControl &servo, float target_position);
void collect_block();
void move_to_pose_watering(const Pose &target_pose, AutoControlMode next_mode, float max_translation_speed = 0.0f);

// std::atomic<int> sw0 = 0;
// std::atomic<int> sw1 = 0;
// std::atomic<int> sw2 = 0;
volatile GPIO_PinState sw0 = GPIO_PIN_RESET;
volatile GPIO_PinState sw1 = GPIO_PIN_RESET;
volatile GPIO_PinState sw2 = GPIO_PIN_RESET;
extern "C" void app_main() {
  hlpuart1.Init.BaudRate = 115200;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK) {
    Error_Handler();
  }
  halx::driver::enable_stdout(debug_uart);

  uart4.start();
  uart5.start();
  debug_uart.start();

  std::printf("GPIO test started\r\n");

  motor1_encoder.start();
  motor2_encoder.start();
  motor3_encoder.start();
  x_encoder.start();
  y_encoder.start();

  motor1.start();
  motor2.start();
  motor3.start();

  imu.start();

  block_holder_servo.start();
  watering_can_servo.start();

  ST_TIM<&htim6>::register_period_elapsed_callback(timer_callback, nullptr);
  ST_TIM<&htim6>::start_base_it();

  while (true) {
    if (auto euler = imu.get_euler()) {
      imu_yaw = std::get<0>(*euler);
    }

    block_holder_servo.update();
    watering_can_servo.update();

    // printf("x: %f, y: %f, yaw: %f, block_pos: %f, watering_pos: %f\n\r", debug_pose_x.load(), debug_pose_y.load(),
    //        debug_pose_yaw.load(), block_holder_servo.get_position(), watering_can_servo.get_position());
    // printf("block_holder_pos %d\n\r", static_cast<int>(block_holder_servo.get_position()));
    // printf("yaw %f\n\r", debug_pose_yaw.load());
    // printf("%f %f %f", motor1_encoder.get_position(), motor2_encoder.get_position(), motor3_encoder.get_position());
    // printf("SW0=%d SW1=%d SW2=%d\r\n", static_cast<int>(sw0), static_cast<int>(sw1), static_cast<int>(sw2));
    printf("X: %f Y: %f YAW: %f\r\n", debug_pose_x.load(), debug_pose_y.load(), debug_pose_yaw.load());
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
  sw1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
  sw0 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);
  sw2 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14);
  update_localization();

  // if (sw0 == 0) {
  //   block_holder_servo.set_position(BLOCK_HOLDER_OPEN_POSITION);
  // } else if (sw0 == 1) {
  //   block_holder_servo.set_position(BLOCK_HOLDER_CLOSED_POSITION);
  // }
  // if (sw1 == 0) {
  //   watering_can_servo.set_position(WATERING_CAN_PULL_POSITION);

  // } else if (sw1 == 1) {
  //   watering_can_servo.set_position(WATERING_CAN_COLLECT_POSITION);
  // }
  if (ps3.get_key_down(PS3Key::SELECT)) {
    competition_running = false;
    stop_drive_wheels_for_pause();
    return;
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
    if (ps3.get_key_down(PS3Key::CIRCLE)) {
      robot_pose = R2_START_POSE;
      competition_ticks = 0;
      competition_running = true;
      stop_drive_wheels();
      set_auto_control_mode(AutoControlMode::GARDEN_TO_B);
    }
    // // メモ　デバッグするときは下のコメントアウトを外してset_auto_control_modeをコメントアウトする
    if (ps3.get_key_down(PS3Key::LEFT)) {
      block_holder_servo.set_position(BLOCK_HOLDER_OPEN_POSITION);
    }
    if (ps3.get_key_down(PS3Key::RIGHT)) {
      block_holder_servo.set_position(BLOCK_HOLDER_CLOSED_POSITION);
    }
    if (ps3.get_key_down(PS3Key::UP)) {
      watering_can_servo.set_position(WATERING_CAN_PULL_POSITION);
    }
    if (ps3.get_key_down(PS3Key::DOWN)) {
      watering_can_servo.set_position(WATERING_CAN_COLLECT_POSITION);
    }
    if (ps3.get_key(PS3Key::R2)) {
      block_holder_servo.set_position(block_holder_servo.get_position() + 10);
    }
    if (ps3.get_key(PS3Key::L2)) {
      block_holder_servo.set_position(block_holder_servo.get_position() - 10);
    }

    Velocity velocity;
    velocity.x = -0.5f * ps3.get_axis(PS3Axis::LEFT_X);
    velocity.y = -0.5f * ps3.get_axis(PS3Axis::LEFT_Y);
    velocity.yaw = -(std::numbers::pi / 2.0f) * ps3.get_axis(PS3Axis::RIGHT_X); // 反時計回りに正となるように符号を反転
    drive_wheels(velocity);
    break;
  }
  // move_to_pose(行く場所, 次の動作)
  // move_servo(動かすサーボ, set_position)
  case AutoControlMode::START_TO_C:
    move_to_pose(WAREHOUSE_C_WAIT_POSE, AutoControlMode::ENTER_WAREHOUSE_C, 0.0f, WAREHOUSE_ENTRY_POSITION_TOLERANCE,
                 WAREHOUSE_ENTRY_POSITION_TOLERANCE);

    break;

  case AutoControlMode::ENTER_WAREHOUSE_C:
    block_holder_servo.set_position(BLOCK_HOLDER_OPEN_POSITION);
    watering_can_servo.set_position(WATERING_CAN_COLLECT_POSITION);
    move_to_pose(WAREHOUSE_C_POSE, AutoControlMode::GET_BLACK_BLOCK, WAREHOUSE_ENTRY_MAX_SPEED,
                 WAREHOUSE_ENTRY_POSITION_TOLERANCE, WAREHOUSE_ENTRY_POSITION_TOLERANCE);
    break;

  case AutoControlMode::GET_BLACK_BLOCK:
    watering_can_servo.set_position(WATERING_CAN_PULL_POSITION);
    collect_block();
    waiting_ticks_mecha = 0;
    set_auto_control_mode(AutoControlMode::WAIT_GET_BLACK_BLOCK);
    break;

  case AutoControlMode::WAIT_GET_BLACK_BLOCK:
    wait_for_mecha(AutoControlMode::EXIT_WAREHOUSE_C);
    break;

  case AutoControlMode::EXIT_WAREHOUSE_C:
    move_to_position_without_rotation(WAREHOUSE_C_EXIT_POSE, AutoControlMode::ROTATE_AFTER_EXIT_WAREHOUSE_C);
    break;

  case AutoControlMode::ROTATE_AFTER_EXIT_WAREHOUSE_C:
    rotate_without_translation(WAREHOUSE_C_EXIT_ROTATED_POSE, AutoControlMode::C_TO_GARDEN);
    break;

  case AutoControlMode::C_TO_GARDEN:
    move_to_pose(GARDEN_BLACK_BLOCK_POSE, AutoControlMode::PUT_BLACK_BLOCK);
    break;

  case AutoControlMode::PUT_BLACK_BLOCK:
    move_servo(block_holder_servo, BLOCK_HOLDER_OPEN_POSITION);
    waiting_ticks_mecha = 0;
    set_auto_control_mode(AutoControlMode::WAIT_PUT_BLACK_BLOCK);
    break;

  case AutoControlMode::WAIT_PUT_BLACK_BLOCK:
    wait_for_mecha(AutoControlMode::BACK_FROM_BLACK_BLOCK_GARDEN);
    break;

  case AutoControlMode::BACK_FROM_BLACK_BLOCK_GARDEN:
    move_to_pose(GARDEN_BLACK_BLOCK_BACK_POSE, AutoControlMode::GARDEN_TO_B);
    break;

  case AutoControlMode::GARDEN_TO_B:
    move_to_pose(WAREHOUSE_B_WAIT_POSE, AutoControlMode::ENTER_WAREHOUSE_B, 0.0f, WAREHOUSE_ENTRY_POSITION_TOLERANCE,
                 WAREHOUSE_ENTRY_POSITION_TOLERANCE);
    break;

  case AutoControlMode::ENTER_WAREHOUSE_B:
    block_holder_servo.set_position(BLOCK_HOLDER_OPEN_POSITION);
    move_to_pose(WAREHOUSE_B_POSE, AutoControlMode::GET_WHITE_BLOCK, WAREHOUSE_ENTRY_MAX_SPEED,
                 WAREHOUSE_ENTRY_POSITION_TOLERANCE, WAREHOUSE_ENTRY_POSITION_TOLERANCE);
    break;

  case AutoControlMode::GET_WHITE_BLOCK:
    move_servo(block_holder_servo, BLOCK_HOLDER_CLOSED_POSITION);
    waiting_ticks_mecha = 0;
    set_auto_control_mode(AutoControlMode::WAIT_GET_WHITE_BLOCK);
    break;

  case AutoControlMode::WAIT_GET_WHITE_BLOCK:
    wait_for_mecha(AutoControlMode::EXIT_WAREHOUSE_B);
    break;

  case AutoControlMode::EXIT_WAREHOUSE_B:
    move_to_position_without_rotation(WAREHOUSE_B_EXIT_POSE, AutoControlMode::ROTATE_AFTER_EXIT_WAREHOUSE_B);
    break;

  case AutoControlMode::ROTATE_AFTER_EXIT_WAREHOUSE_B:
    rotate_without_translation(WAREHOUSE_B_EXIT_ROTATED_POSE, AutoControlMode::B_TO_GARDEN);

    break;

  case AutoControlMode::B_TO_GARDEN:
    move_to_pose(GARDEN_WHITE_BLOCK_POSE, AutoControlMode::PUT_WHITE_BLOCK);
    break;

  case AutoControlMode::PUT_WHITE_BLOCK:
    move_servo(block_holder_servo, BLOCK_HOLDER_OPEN_POSITION);
    waiting_ticks_mecha = 0;
    set_auto_control_mode(AutoControlMode::WAIT_PUT_WHITE_BLOCK);
    break;

  case AutoControlMode::WAIT_PUT_WHITE_BLOCK:
    wait_for_mecha(AutoControlMode::BACK_FROM_WHITE_BLOCK_GARDEN);
    break;

  case AutoControlMode::BACK_FROM_WHITE_BLOCK_GARDEN:
    move_to_pose(GARDEN_WHITE_BLOCK_BACK_POSE, AutoControlMode::ROTATE_AFTER_WHITE_BLOCK);
    break;

  case AutoControlMode::ROTATE_AFTER_WHITE_BLOCK:
    move_to_pose(GARDEN_WHITE_BLOCK_EXIT_POSE, AutoControlMode::WAIT_FOR_WATERING);
    break;

  case AutoControlMode::WAIT_FOR_WATERING:
    waiting_ticks = waiting_ticks + 1;
    // 邪魔だったら待機場所を設定してもいいかも
    stop_drive_wheels();
    if (waiting_ticks >= WATERING_START_TICKS) {
      set_auto_control_mode(AutoControlMode::WATERING_TO_A_RELAY);
    }
    break;

  case AutoControlMode::WATERING_TO_A_RELAY:
    move_to_pose_watering(WATERING_WAREHOUSE_A_RELAY, AutoControlMode::WATERING_A_RELAY_TO_A);
    break;

  case AutoControlMode::WATERING_A_RELAY_TO_A:
    move_to_pose_watering(WATERING_WAREHOUSE_A, AutoControlMode::WATERING_A_TO_A_RELAY);
    break;

  case AutoControlMode::WATERING_A_TO_A_RELAY:
    move_to_pose_watering(WATERING_WAREHOUSE_A_RELAY, AutoControlMode::WATERING_A_RELAY_TO_GARDEN);
    break;

  case AutoControlMode::WATERING_A_RELAY_TO_GARDEN:
    move_to_pose_watering(WATERING_GARDEN, AutoControlMode::WATERING_GARDEN_TO_C_RELAY);
    break;

  case AutoControlMode::WATERING_GARDEN_TO_C_RELAY:
    move_to_pose_watering(WATERING_WAREHOUSE_C_RELAY, AutoControlMode::WATERING_C_RELAY_TO_C);
    break;

  case AutoControlMode::WATERING_C_RELAY_TO_C:
    move_to_pose_watering(WATERING_WAREHOUSE_C, AutoControlMode::WATERING_C_TO_C_RELAY);
    break;

  case AutoControlMode::WATERING_C_TO_C_RELAY:
    move_to_pose_watering(WATERING_WAREHOUSE_C_RELAY, AutoControlMode::WATERING_C_RELAY_TO_GARDEN);
    break;

  case AutoControlMode::WATERING_C_RELAY_TO_GARDEN:
    move_to_pose_watering(WATERING_GARDEN, AutoControlMode::WATERING_TO_A_RELAY);
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

void wait_for_mecha(AutoControlMode next_mode) {
  stop_drive_wheels();
  if (++waiting_ticks_mecha >= WAIT_TICKS_MECHA) {
    waiting_ticks_mecha = 0;
    set_auto_control_mode(next_mode);
  }
}

void move_to_pose(const Pose &target_pose, AutoControlMode next_mode, float max_translation_speed,
                  float x_position_tolerance, float y_position_tolerance) {
  const float delta_x = target_pose.x - robot_pose.x;
  const float delta_y = target_pose.y - robot_pose.y;
  const float delta_yaw = std::remainder(target_pose.yaw - robot_pose.yaw, 2.0f * std::numbers::pi);

  if (std::abs(delta_x) <= x_position_tolerance && std::abs(delta_y) <= y_position_tolerance &&
      std::abs(delta_yaw) <= SEQUENCE_YAW_TOLERANCE) {
    stop_drive_wheels();
    set_auto_control_mode(next_mode);
    return;
  }

  Velocity target_velocity = calculate_velocity(robot_pose, target_pose);
  if (max_translation_speed > 0.0f) {
    const float translation_speed_squared =
        target_velocity.x * target_velocity.x + target_velocity.y * target_velocity.y;
    const float max_translation_speed_squared = max_translation_speed * max_translation_speed;
    if (translation_speed_squared > max_translation_speed_squared) {
      const float speed_ratio = max_translation_speed / std::sqrt(translation_speed_squared);
      target_velocity.x *= speed_ratio;
      target_velocity.y *= speed_ratio;
    }
  }

  drive_wheels(target_velocity);
}

void move_to_position_without_rotation(const Pose &target_pose, AutoControlMode next_mode) {
  const float delta_x = target_pose.x - robot_pose.x;
  const float delta_y = target_pose.y - robot_pose.y;

  if (std::abs(delta_x) <= SEQUENCE_X_POSITION_TOLERANCE && std::abs(delta_y) <= SEQUENCE_Y_POSITION_TOLERANCE) {
    stop_drive_wheels();
    set_auto_control_mode(next_mode);
    return;
  }

  Velocity target_velocity = calculate_velocity(robot_pose, target_pose);
  target_velocity.yaw = 0.0f;
  drive_wheels(target_velocity);
}

void rotate_without_translation(const Pose &target_pose, AutoControlMode next_mode) {
  const float delta_yaw = std::remainder(target_pose.yaw - robot_pose.yaw, 2.0f * std::numbers::pi);

  if (std::abs(delta_yaw) <= SEQUENCE_YAW_TOLERANCE) {
    stop_drive_wheels();
    set_auto_control_mode(next_mode);
    return;
  }

  const Pose rotation_target{robot_pose.x, robot_pose.y, target_pose.yaw};
  Velocity target_velocity = calculate_velocity(robot_pose, rotation_target);
  target_velocity.x = 0.0f;
  target_velocity.y = 0.0f;
  drive_wheels(target_velocity);
}

void move_to_pose_watering(const Pose &target_pose, AutoControlMode next_mode, float max_translation_speed) {
  const float delta_x = target_pose.x - robot_pose.x;
  const float delta_y = target_pose.y - robot_pose.y;
  const float delta_yaw = std::remainder(target_pose.yaw - robot_pose.yaw, 2.0f * std::numbers::pi);

  if (std::abs(delta_x) <= SEQUENCE_X_POSITION_TOLERANCE && std::abs(delta_y) <= SEQUENCE_Y_POSITION_TOLERANCE &&
      std::abs(delta_yaw) <= SEQUENCE_YAW_TOLERANCE) {
    set_auto_control_mode(next_mode);
    return;
  }

  Velocity target_velocity = calculate_velocity(robot_pose, target_pose);
  if (max_translation_speed > 0.0f) {
    const float translation_speed_squared =
        target_velocity.x * target_velocity.x + target_velocity.y * target_velocity.y;
    const float max_translation_speed_squared = max_translation_speed * max_translation_speed;
    if (translation_speed_squared > max_translation_speed_squared) {
      const float speed_ratio = max_translation_speed / std::sqrt(translation_speed_squared);
      target_velocity.x *= speed_ratio;
      target_velocity.y *= speed_ratio;
    }
  }

  drive_wheels(target_velocity);
}
void move_servo(FeetechPositionControl &servo, float target_position) {
  stop_drive_wheels();
  servo.set_position(target_position);
}

void collect_block() {
  stop_drive_wheels();
  block_holder_servo.set_position(BLOCK_HOLDER_CLOSED_POSITION);
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
  const float yaw_error = std::remainder(target_pose.yaw - now_pose.yaw, 2.0f * std::numbers::pi);
  world_velocity.yaw = p2p_yaw_pid.solve(yaw_error);

  Velocity robot_velocity;
  robot_velocity.x = world_velocity.x * std::cos(now_pose.yaw) + world_velocity.y * std::sin(now_pose.yaw);
  robot_velocity.y = world_velocity.y * std::cos(now_pose.yaw) - world_velocity.x * std::sin(now_pose.yaw);
  robot_velocity.yaw = world_velocity.yaw;

  return robot_velocity;
}

Velocity limit_acceleration(const Velocity &target_velocity) {
  const float max_idou_delta = MAX_IDOU_ACCEL * CONTROL_DT; // velocity
  float delta_x = target_velocity.x - pre_velocity.x;
  float delta_y = target_velocity.y - pre_velocity.y;
  const float idou_delta = std::hypot(delta_x, delta_y); // 二条和の平方根

  if (idou_delta > max_idou_delta) {
    const float ratio = max_idou_delta / idou_delta;
    delta_x *= ratio;
    delta_y *= ratio;
  }

  pre_velocity.x += delta_x;
  pre_velocity.y += delta_y;

  const float max_yaw_delta = MAX_YAW_ACCEL * CONTROL_DT;
  pre_velocity.yaw += std::clamp(target_velocity.yaw - pre_velocity.yaw, -max_yaw_delta,
                                 max_yaw_delta); // 足し算って感じじゃないのでclamp

  return pre_velocity;
}

void drive_wheels(const Velocity &target_velocity) {
  const Velocity velocity = limit_acceleration(target_velocity);
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
  pre_velocity = {0.0f, 0.0f, 0.0f};

  motor1.set_output(0.0f);
  motor2.set_output(0.0f);
  motor3.set_output(0.0f);

  motor1_pid = PIDController(DRIVE_WHEEL_PID_PARAMS, CONTROL_DT);
  motor2_pid = PIDController(DRIVE_WHEEL_PID_PARAMS, CONTROL_DT);
  motor3_pid = PIDController(DRIVE_WHEEL_PID_PARAMS, CONTROL_DT);
}

void stop_drive_wheels_for_pause() {
  stop_drive_wheels();
  set_auto_control_mode(AutoControlMode::MANUAL);
}
