#ifndef Core_Inc_controller_hpp_
#define Core_Inc_controller_hpp_
#include <functional>

#include "parts.hpp"
namespace state {
template <typename T, class STATUS, class PID>
class Controller {
 private:
  parts::wheel<std::unique_ptr<PID>, std::unique_ptr<PID>> speed, ang_vel;
  parts::wheel<T, T> motor_duty;
  T tar_speed = 0, accel = 0;
  T tar_ang_vel = 0, ang_acc = 0, tar_degree = 0;
  T max_speed = 0, max_ang_vel = 0, max_degree = 0;

 public:
  STATUS status;
  parts::Run_mode_t run_mode;
  Controller() : status() {
    speed.left = std::make_unique<PID>(22.47f, 0.0366f, 0.0f);
    speed.right = std::make_unique<PID>(22.47f, 0.0366f, 0.0f);
    ang_vel.right = std::make_unique<PID>(1.52f, 2.25f, 0.000f);
    ang_vel.left = std::make_unique<PID>(1.52f, 2.25f, 0.000f);
    motor_duty.left = 0;
    motor_duty.right = 0;
  }
  void update() {
    motor_duty.left = 0;
    motor_duty.right = 0;
    // motor_duty.left += speed.left->update(tar_speed, status.speed);
    // motor_duty.right += speed.right->update(tar_speed, status.speed);
    motor_duty.left += ang_vel.left->update(tar_ang_vel, status.I_ang_vel);
    motor_duty.right -= ang_vel.right->update(tar_ang_vel, status.I_ang_vel);
    if (motor_duty.left >= 1000) {
      motor_duty.left = 999;
    } else if (motor_duty.left <= -1000) {
      motor_duty.left = -999;
    }
    if (motor_duty.right >= 1000) {
      motor_duty.right = 999;
    } else if (motor_duty.right <= -1000) {
      motor_duty.right = -999;
    }
    // printf("motor_duty.left = %f, motor_duty.right = %f\r\n", ang_vel.left->update(tar_ang_vel, status.ang_vel), ang_vel.right->update(tar_ang_vel, status.ang_vel));
    //    printf("angvel:%f\r\n", status.ang_vel);
  }

  template <class MOTOR, void (MOTOR::*DRIVEFn)(int16_t)>
  void drive_motor(MOTOR &left_motor, MOTOR &right_motor, int8_t left_dir, int8_t right_dir) {
    (left_motor.*DRIVEFn)(left_dir * motor_duty.left);
    (right_motor.*DRIVEFn)(right_dir * motor_duty.right);
  }
  void generate_tar_speed() {
    // 直線の場合の目標速度生成
    if (run_mode == parts::Run_mode_t::STRAIGHT_MODE) {
      tar_speed += accel * 0.001f;  // 目標速度を設定加速度で更新
      // 最高速度制限
      if (tar_speed > max_speed) {
        tar_speed = max_speed;  // 目標速度を設定最高速度に設定
      }
    } else if (run_mode == parts::Run_mode_t::TURN_MODE) {
      // 車体中心速度更新
      tar_speed += accel * 0.001f;
      // 最高速度制限
      if (tar_speed > max_speed) {
        tar_speed = max_speed;  // 目標速度を設定最高速度に設定
      }

      // 角加速度更新
      tar_ang_vel += ang_acc / 1000.0;  // 目標角速度を設定加速度で更新
      tar_degree += (tar_ang_vel * 180.0 / PI) / 1000.0;

      // 最高角速度制限
      if (std::abs(tar_ang_vel) > std::abs(max_ang_vel)) {
        tar_ang_vel = max_ang_vel;  // 目標速度を設定最高速度に設定
      }
      if (std::abs(tar_degree) > std::abs(max_degree)) {
        tar_degree = max_degree;
      }
    }
  }

  void straight(T len, T acc, T max_sp, T end_sp) {
    // 走行モードを直線にする
    run_mode = parts::Run_mode_t::STRAIGHT_MODE;
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
      while (((len_target - 10) - status.len_mouse) > 1000.0 * ((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed)) / (float)(2.0 * accel))
        ;
      // 減速処理開始
      accel = -acc;                                // 減速するために加速度を負の値にする
      while (status.len_mouse < len_target - 1) {  // 停止したい距離の少し手前まで継続
        // 一定速度まで減速したら最低駆動トルクで走行
        if (tar_speed <= 0.1) {  // 目標速度が最低速度になったら、加速度を0にする
          accel = 0;
          tar_speed = 0.1;
        }
      }
      accel = 0;
      tar_speed = 0;
      // 速度が0以下になるまで逆転する
      while (speed >= 0.0)
        ;

    } else {
      // 減速処理を始めるべき位置まで加速、定速区間を続行
      while (((len_target - 10) - status.len_mouse) > 1000.0 * ((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed)) / (float)(2.0 * accel))
        ;

      // 減速処理開始
      accel = -acc;                            // 減速するために加速度を負の値にする
      while (status.len_mouse < len_target) {  // 停止したい距離の少し手前まで継続
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
    status.len_mouse = 0;
  }

  // positive: left, negative: right
  void turn(const int deg, float ang_accel, float max_ang_velocity) {
    HAL_Delay(500);
    if (deg < 0) {
      ang_accel = ang_accel < 0 ? ang_accel : -ang_accel;
      max_ang_velocity = max_ang_velocity < 0 ? max_ang_velocity : -max_ang_velocity;
    } else {
      ang_accel = ang_accel >= 0 ? ang_accel : -ang_accel;
      max_ang_velocity = max_ang_velocity >= 0 ? max_ang_velocity : -max_ang_velocity;
    }
    tar_degree = 0;

    float local_degree = 0;
    accel = 0;
    tar_speed = 0;
    tar_ang_vel = 0;
    // 走行モードをスラロームモードにする
    run_mode = parts::Run_mode_t::TURN_MODE;

    // 車体の現在角度を取得
    local_degree = status.degree;
    tar_degree = 0;
    ang_acc = ang_accel;
    max_ang_vel = max_ang_velocity;
    max_degree = deg;
    // 角加速度、加速度、最高角速度設定
    while (std::abs(deg - (status.degree - local_degree)) * PI / 180.0 > std::abs(tar_ang_vel * tar_ang_vel / (2.0 * ang_accel)))
      ;

    // BEEP();
    // 角減速区間に入るため、角加速度設定
    while (abs(status.degree - local_degree) < abs(max_degree)) {
      if (tar_ang_vel < (PI / 10.0f)) {
        ang_acc = 0;
        tar_ang_vel = (PI / 10.0f);
      }
    }

    ang_acc = 0;
    tar_ang_vel = 0;
    tar_degree = max_degree;

    while (ang_vel >= 0.05 || ang_vel <= -0.05)
      ;

    tar_ang_vel = 0;
    ang_acc = 0;
    // 現在距離を0にリセット
    status.len_mouse = 0;
    HAL_Delay(500);
  }
};
}  // namespace state

#endif  // CORE_INC_CONTROLLER_HPP_
