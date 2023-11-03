#ifndef CORE_INC_CONTROLLER_HPP_
#define CORE_INC_CONTROLLER_HPP_
#include <cfloat>
#include <cmath>
#include <functional>
#include <memory>

#include "parts.hpp"
namespace state {
template <typename T, class STATUS, class PID>
class Controller {
 private:
  parts::wheel<std::unique_ptr<PID>, std::unique_ptr<PID>> speed = {std::make_unique<PID>(0.0041024f, 0.067247f, 0.0f, 0.0f),
                                                                    std::make_unique<PID>(0.0041024f, 0.067247f, 0.0f, 0.0f)},
                                                           ang_vel = {std::make_unique<PID>(0.0041024f, 0.067247f, 0.0f, 0.0f),
                                                                      std::make_unique<PID>(0.0041024f, 0.067247f, 0.0f, 0.0f)},
                                                           ang = {std::make_unique<PID>(0.5f, 0.05f, 0.001f, 0.0f), std::make_unique<PID>(0.5f, 0.05f, 0.001f, 0.0f)};
  std::unique_ptr<PID> wall = std::make_unique<PID>(0.01f, 0.00f, 0.000f, 0.0f);

  parts::wheel<T, T> motor_duty = {0, 0};
  T tar_speed = 0, accel = 0;
  float tar_ang_vel = 0, ang_acc = 0, tar_degree = 0;
  float max_speed = 0, max_ang_vel = 0, max_degree = 0;
  static constexpr float turn_min_vel = 36.0F;
  static constexpr float turn_vel_error = 3.0F;
  static constexpr float len_start_dec_vel = 10.0F;  // mm
  static constexpr float min_speed = 100.0F;         // mm/s

  parts::RunModeT run_mode = parts::RunModeT::STOP_MODE;

 public:
  STATUS status;  // NOLINT
  bool wall_control = true;
  Controller() : status() {}
  void update() {
    generate_tar_speed();
    motor_duty.left = 0;
    motor_duty.right = 0;

    if (run_mode == parts::RunModeT::STRAIGHT_MODE) {
      if (wall_control) {
        parts::wheel<T, T> wall_sensor_error = status.get_wall_sensor_error();
        tar_ang_vel += wall->update(0, wall_sensor_error.left - wall_sensor_error.right);
      }
    }
    motor_duty.left += speed.left->update(tar_speed, status.get_speed());
    motor_duty.right += speed.right->update(tar_speed, status.get_speed());

    motor_duty.left -= ang_vel.left->update(tar_ang_vel, status.get_ang_vel());
    motor_duty.right += ang_vel.right->update(tar_ang_vel, status.get_ang_vel());
    if (run_mode == parts::RunModeT::STRAIGHT_MODE) {
      tar_ang_vel = 0;
    }
    if (run_mode == parts::RunModeT::STOP_MODE) {
      motor_duty.left -= ang.left->update(0.0F, status.get_ang());
      motor_duty.right += ang.right->update(0.0F, status.get_ang());
    }
  }

  template <class MOTOR, void (MOTOR::*DRIVEFn)(float)>
  void drive_motor(MOTOR &left_motor, MOTOR &right_motor, const int8_t left_dir, const int8_t right_dir) {
    (left_motor.*DRIVEFn)(left_dir * static_cast<float>(motor_duty.left));
    (right_motor.*DRIVEFn)(right_dir * static_cast<float>(motor_duty.right));
  }
  void generate_tar_speed() {
    // 直線の場合の目標速度生成
    if (run_mode == parts::RunModeT::STRAIGHT_MODE) {
      tar_speed += accel * 0.001F;  // 目標速度を設定加速度で更新
      // 最高速度制限
      if (std::abs(tar_speed) > std::abs(max_speed)) {
        tar_speed = max_speed;  // 目標速度を設定最高速度に設定
      }

    } else if (run_mode == parts::RunModeT::TURN_MODE) {
      // // 車体中心速度更新
      // tar_speed += accel * 0.001;
      // // 最高速度制限
      // if (tar_speed > max_speed) {
      //   tar_speed = max_speed;  // 目標速度を設定最高速度に設定
      // }

      // 角加速度更新
      tar_ang_vel += ang_acc / 1000;  // 目標角速度を設定加速度で更新
      // tar_degree += tar_ang_vel / 1000;

      // 最高角速度制限
      if (std::abs(tar_ang_vel) > std::abs(max_ang_vel)) {
        tar_ang_vel = max_ang_vel;  // 目標速度を設定最高速度に設定
      }
      // if (std::abs(tar_degree) > std::abs(max_degree)) {
      //   tar_degree = max_degree;
      // }
    }
  }

  void straight(T len, T acc, T max_sp, T end_sp) {  // mm
    // 走行モードを直線にする
    run_mode = parts::RunModeT::STRAIGHT_MODE;
    // 壁制御を有効にする

    const T len_target = len;
    // 目標速度を設定
    const T end_speed = end_sp;
    // 加速度を設定
    accel = acc;
    // 最高速度を設定
    max_speed = max_sp;
    // 減速処理を始めるべき位置まで加速、定速区間を続行
    while (((len_target - len_start_dec_vel) - status.get_len_mouse()) /*mm*/ >
           (static_cast<float>(tar_speed * tar_speed) - static_cast<float>(end_speed * end_speed)) / static_cast<float>(2 * accel) /*mm*/) {
      HAL_Delay(1);
    }
    // 減速処理開始

    float end_tar_speed = (std::fabs(end_speed) < FLT_EPSILON ? min_speed : end_speed);
    accel = -acc;
    while (std::abs(status.get_len_mouse()) < std::abs(len_target - 1)) {  // 停止したい距離の少し手前まで継続
      // 一定速度まで減速したら最低駆動トルクで走行

      if (tar_speed <= end_tar_speed) {  // 目標速度が最低速度になったら、加速度を0にする
        accel = 0;
        tar_speed = end_tar_speed;
      }

      HAL_Delay(1);
    }
    // 加速度を0にする
    accel = 0;
    tar_speed = (std::abs(end_speed) < FLT_EPSILON ? 0 : end_speed);

    if (std::abs(end_speed) < FLT_EPSILON) {
      while (status.get_speed() > FLT_EPSILON) {
        HAL_Delay(1);
      }
    }

    // 現在距離を0にリセット
    status.reset();
    if (std::abs(end_speed) < FLT_EPSILON) run_mode = parts::RunModeT::STOP_MODE;
    speed.left->reset();
    speed.right->reset();
    HAL_Delay(1);
  }

  // positive: left, negative: right
  void turn(const float deg, float ang_accel, float max_ang_velocity) {
    ang_accel = std::abs(ang_accel);
    max_ang_velocity = std::abs(max_ang_velocity);
    if (deg < 0) {
      ang_accel = -ang_accel;
      max_ang_velocity = -max_ang_velocity;
    }
    // tar_degree = 0;

    float local_degree = 0;
    accel = 0;
    tar_speed = 0;
    tar_ang_vel = 0;
    // 走行モードをスラロームモードにする
    run_mode = parts::RunModeT::TURN_MODE;

    // 車体の現在角度を取得
    local_degree = status.get_ang();
    // tar_degree = 0;
    ang_acc = ang_accel;
    max_ang_vel = max_ang_velocity;
    max_degree = deg;
    // 角加速度、加速度、最高角速度設定
    while (std::abs(deg - (status.get_ang() - local_degree)) > std::abs(tar_ang_vel * tar_ang_vel / (2 * ang_accel))) {
      HAL_Delay(1);
    }

    // BEEP();
    // 角減速区間に入るため、角加速度設定

    ang_acc = -ang_accel;

    while (std::abs(status.get_ang() - local_degree) < std::abs(max_degree)) {
      if (std::abs(tar_ang_vel) < turn_min_vel) {
        ang_acc = 0;
        tar_ang_vel = (tar_ang_vel >= 0 ? turn_min_vel : -turn_min_vel);
      }
      HAL_Delay(1);
    }

    ang_acc = 0;
    tar_ang_vel = 0;

    // tar_degree = max_degree;

    while (std::abs(status.get_ang_vel()) >= std::abs(turn_vel_error)) {  // NOLINT
      HAL_Delay(1);
    }

    tar_ang_vel = 0;
    ang_acc = 0;
    // 現在距離を0にリセット
    status.reset();

    run_mode = parts::RunModeT::STOP_MODE;
    ang_vel.left->reset();
    ang_vel.right->reset();
    HAL_Delay(1);
  }
};
}  // namespace state

#endif  // CORE_INC_CONTROLLER_HPP_
