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

void timer_callback(void *);

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

  while (!imu.ping()) {
  }
  imu.start();

  ST_TIM<&htim6>::register_period_elapsed_callback(timer_callback, nullptr);
  ST_TIM<&htim6>::start_base_it();

  while (true) {
    if (auto euler = imu.get_euler()) {
      imu_yaw = std::get<0>(*euler);
    }

    printf("yaw: %f\r\n", imu_yaw.load());

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

  static PIDController motor1_pid(DRIVE_WHEEL_PID_PARAMS, CONTROL_DT);
  static PIDController motor2_pid(DRIVE_WHEEL_PID_PARAMS, CONTROL_DT);
  static PIDController motor3_pid(DRIVE_WHEEL_PID_PARAMS, CONTROL_DT);

  float vx = 2.0f * ps3.get_axis(PS3Axis::LEFT_X);
  float vy = 2.0f * ps3.get_axis(PS3Axis::LEFT_Y);

  float motor1_target_rps =
      -vx * std::sin(DRIVE_WHEEL_THETA_1) + vy * std::cos(DRIVE_WHEEL_THETA_1);
  float motor2_target_rps =
      -vx * std::sin(DRIVE_WHEEL_THETA_2) + vy * std::cos(DRIVE_WHEEL_THETA_2);
  float motor3_target_rps =
      -vx * std::sin(DRIVE_WHEEL_THETA_3) + vy * std::cos(DRIVE_WHEEL_THETA_3);

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
