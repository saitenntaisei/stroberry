#ifndef CORE_INC_CONTROLLER_HPP_
#define CORE_INC_CONTROLLER_HPP_
#include <cmath>
#include <functional>
#include <memory>

#include "parts.hpp"
namespace state {
template <typename T, class STATUS, class PID>
class Controller {
 private:
  parts::wheel<std::unique_ptr<PID>, std::unique_ptr<PID>> speed = {std::make_unique<PID>(0.0031484f, 0.011501f, 2.113f / 1e5f, 0.12267f),
                                                                    std::make_unique<PID>(0.0031484f, 0.011501f, 2.113f / 1e5f, 0.12267f)},
                                                           ang_vel = {std::make_unique<PID>(0.0031484f, 0.011501f, 2.113f / 1e5f, 0.12267f),
                                                                      std::make_unique<PID>(0.0031484f, 0.011501f, 2.113f / 1e5f, 0.12267f)},
                                                           ang = {std::make_unique<PID>(0.5f, 0.05f, 0.001f, 0.0f), std::make_unique<PID>(0.5f, 0.05f, 0.001f, 0.0f)};

  parts::wheel<T, T> motor_duty = {0, 0};
  T tar_speed = 0, accel = 0;
  float tar_ang_vel = 0, ang_acc = 0, tar_degree = 0;
  float max_speed = 0, max_ang_vel = 0, max_degree = 0;
  static constexpr float turn_min_vel = 18.0F;
  static constexpr float turn_vel_error = 3.0F;

  parts::RunModeT run_mode = parts::RunModeT::STOP_MODE;

 public:
  STATUS status;  // NOLINT
  Controller() : status() {}
  void update() {
    generate_tar_speed();
    motor_duty.left = 0;
    motor_duty.right = 0;

    motor_duty.left += speed.left->update(tar_speed, status.get_speed());
    motor_duty.right += speed.right->update(tar_speed, status.get_speed());
    motor_duty.left += ang_vel.left->update(tar_ang_vel, status.get_ang_vel());

    motor_duty.right -= ang_vel.right->update(tar_ang_vel, status.get_ang_vel());
    if (run_mode == parts::RunModeT::STOP_MODE) {
      motor_duty.right -= ang_vel.right->update(0.0F, status.get_ang());
      motor_duty.left += ang.left->update(0.0F, status.get_ang());
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
      // tar_speed += accel * 0.001;  // 目標速度を設定加速度で更新
      // // 最高速度制限
      // if (tar_speed > max_speed) {
      //   tar_speed = max_speed;  // 目標速度を設定最高速度に設定
      // }
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

  void straight(T len, T acc, T max_sp, T end_sp) {
    // 走行モードを直線にする
    run_mode = parts::RunModeT::STRAIGHT_MODE;
    // 壁制御を有効にする
    // con_wall.enable = true;
    // 目標距離をグローバル変数に代入する
    T len_target = len;
    // 目標速度を設定
    T end_speed = end_sp;
    // 加速度を設定
    accel = acc;
    // 最高速度を設定
    max_speed = max_sp;

    if (end_speed == 0) {  // 最終的に停止する場合
      // 減速処理を始めるべき位置まで加速、定速区間を続行
      while (((len_target - 10) - status.get_len_mouse()) >
             1000 * (static_cast<float>(tar_speed * tar_speed) - static_cast<float>(end_speed * end_speed)) / static_cast<float>(2 * accel)) {
      }
      // 減速処理開始
      accel = -acc;                                      // 減速するために加速度を負の値にする
      while (status.get_len_mouse() < len_target - 1) {  // 停止したい距離の少し手前まで継続
        // 一定速度まで減速したら最低駆動トルクで走行
        if (tar_speed <= 0.1) {  // 目標速度が最低速度になったら、加速度を0にする
          accel = 0;
          tar_speed = 0.1;
        }
      }
      accel = 0;
      tar_speed = 0;
      // 速度が0以下になるまで逆転する
      while (speed >= 0.0) {
      }

    } else {
      // 減速処理を始めるべき位置まで加速、定速区間を続行
      while (((len_target - 10) - status.get_len_mouse()) >
             1000 * (static_cast<float>(tar_speed * tar_speed) - static_cast<float>(end_speed * end_speed)) / static_cast<float>(2 * accel)) {
      }

      // 減速処理開始
      accel = -acc;                                  // 減速するために加速度を負の値にする
      while (status.get_len_mouse() < len_target) {  // 停止したい距離の少し手前まで継続
        // 一定速度まで減速したら最低駆動トルクで走行
        if (tar_speed <= end_speed) {  // 目標速度が最低速度になったら、加速度を0にする
          accel = 0;
          // tar_speed = end_speed;
        }
      }
    }
    // 加速度を0にする
    accel = 0;
    // 現在距離を0にリセット
    status.set_len_mouse(0);
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
