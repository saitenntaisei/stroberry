#ifndef CORE_INC_CONTROLLER_HPP_
#define CORE_INC_CONTROLLER_HPP_
#include <cfloat>
#include <cmath>
#include <functional>
#include <memory>

#include "maze_run.hpp"
#include "parts.hpp"
#include "state.hpp"
namespace state {
template <typename T, class STATUS, class PID>
class Controller {
 private:
  parts::wheel<PID, PID> speed = {PID(0.00809f, 0.031819f, 0.00048949f, 0.0f), PID(0.00809f, 0.031819f, 0.00048949f, 0.0f)},
                         front_wall = {PID(0.00005f, 0.00008f, 0.0000014f, 0.0f), PID(0.00005f, 0.00008f, 0.0000014f, 0.0f)},
                         ang = {PID(0.5f, 0.05f, 0.001f, 0.0f), PID(0.5f, 0.05f, 0.001f, 0.0f)};
  PID side_wall = PID(0.004f, 0.000f, 0.0000f, 0.0f);
  PID ang_vel = PID(0.0041024f, 0.067247f, 0.0f, 0.0f);

  parts::wheel<T, T> motor_duty = {0, 0};
  T tar_speed = 0, accel = 0;
  float tar_ang_vel = 0, ang_acc = 0, tar_degree = 0;
  float max_speed = 0, max_ang_vel = 0, max_degree = 0;
  static constexpr float turn_min_vel = 36.0F;
  static constexpr float turn_vel_error = 0.1F;      // 3.0F;
  static constexpr float len_start_dec_vel = 10.0F;  // mm
  static constexpr float min_speed = 100.0F;         // mm/s

  parts::RunModeT run_mode = parts::RunModeT::STOP_MODE;
  bool front_wall_control = false;
  bool side_wall_control = true;
  bool is_enable_front_wall_control = true;

 public:
  state::Status<float> status;  // NOLINT

  Controller() : status() {}
  void set_side_wall_control(bool side_wall_control) { this->side_wall_control = side_wall_control; }
  void set_front_wall_control_permission(bool is_enable_front_wall_control) { this->is_enable_front_wall_control = is_enable_front_wall_control; }
  void reset();
  void update();
  template <class MOTOR, void (MOTOR::*DRIVEFn)(float)>
  void drive_motor(MOTOR &left_motor, MOTOR &right_motor, const std::int8_t left_dir, const std::int8_t right_dir);
  void back_1s();
  void turn(const float deg, float ang_accel, float max_ang_velocity);
  void straight(T len, T acc, T max_sp, T end_sp);
  void generate_tar_speed();
};

template <typename T, class STATUS, class PID>
void Controller<T, STATUS, PID>::reset() {
  speed.left.reset();
  speed.right.reset();
  ang_vel.reset();
  ang.left.reset();
  ang.right.reset();
  side_wall.reset();
  front_wall.left.reset();
  front_wall.right.reset();
}
template <typename T, class STATUS, class PID>
void Controller<T, STATUS, PID>::update() {
  generate_tar_speed();
  motor_duty.left = 0;
  motor_duty.right = 0;

  if (run_mode == parts::RunModeT::STRAIGHT_MODE) {
    if (side_wall_control) {
      parts::wheel<T, T> side_wall_sensor_error = status.get_side_wall_sensor_error();
      parts::wheel<bool, bool> is_side_wall = status.get_is_side_wall_control();
      std::uint8_t n = 1;

      if (!is_side_wall.left || !is_side_wall.right) {
        n = 1;
      }
      if (!is_side_wall.left && !is_side_wall.right) n = 0;

      tar_ang_vel += side_wall.update(0, side_wall_sensor_error.left - side_wall_sensor_error.right) * (float)n;
    }
  }

  if (!front_wall_control) {
    motor_duty.left += speed.left.update(tar_speed, status.get_speed());
    motor_duty.right += speed.right.update(tar_speed, status.get_speed()) * 1.1f;
    float ang_vel_pid = ang_vel.update(tar_ang_vel, status.get_ang_vel());
    motor_duty.left -= ang_vel_pid;
    motor_duty.right += ang_vel_pid;
  }
  if (front_wall_control) {
    parts::wheel<T, T> front_wall_sensor_error = status.get_front_wall_sensor_error();
    motor_duty.left += front_wall.left.update(0, front_wall_sensor_error.left);
    motor_duty.right += front_wall.right.update(0, front_wall_sensor_error.right);
  }
  if (run_mode == parts::RunModeT::STRAIGHT_MODE) {
    tar_ang_vel = 0;
  }
  if (run_mode == parts::RunModeT::STOP_MODE && !front_wall_control) {
    motor_duty.left -= ang.left.update(0.0F, status.get_ang());
    motor_duty.right += ang.right.update(0.0F, status.get_ang());
  }
}

template <typename T, class STATUS, class PID>
template <class MOTOR, void (MOTOR::*DRIVEFn)(float)>
void Controller<T, STATUS, PID>::drive_motor(MOTOR &left_motor, MOTOR &right_motor, const std::int8_t left_dir, const std::int8_t right_dir) {
  (left_motor.*DRIVEFn)(left_dir * static_cast<float>(motor_duty.left));
  (right_motor.*DRIVEFn)(right_dir * static_cast<float>(motor_duty.right));
}
template <typename T, class STATUS, class PID>
void Controller<T, STATUS, PID>::generate_tar_speed() {
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

template <typename T, class STATUS, class PID>
void Controller<T, STATUS, PID>::back_1s() {
  run_mode = parts::RunModeT::STRAIGHT_MODE;
  tar_speed = -100;
  max_speed = -100;
  HAL_Delay(2000);
  tar_speed = 0;
  max_speed = 0;
  status.reset();
  speed.left.reset();
  speed.right.reset();
}

template <typename T, class STATUS, class PID>
void Controller<T, STATUS, PID>::straight(T len, T acc, T max_sp, T end_sp) {  // mm
  // 走行モードを直線にする
  run_mode = parts::RunModeT::STRAIGHT_MODE;
  // 壁制御を有効にする

  const T len_target = len;
  // 目標速度を設定
  const T end_speed = end_sp;
  // 加速度を設定
  if (len_target < 0) {
    acc = -acc;
  }
  if (len_target < 0) {
    max_sp = -max_sp;
  }
  accel = acc;
  // 最高速度を設定
  max_speed = max_sp;
  // 減速処理を始めるべき位置まで加速、定速区間を続行
  while ((len_target - len_start_dec_vel) - status.get_len_mouse() /*mm*/ >
         (static_cast<float>(end_speed * end_speed) - static_cast<float>(tar_speed * tar_speed)) / (static_cast<float>(-2 * accel))) /*mm*/ {
    HAL_Delay(1);
  }
  // 減速処理開始
  bool side_wall_control_tmp = side_wall_control;

  float end_tar_speed = (std::fabs(end_speed) < FLT_EPSILON ? min_speed : end_speed);
  accel = -acc;
  while (std::abs(status.get_len_mouse()) < std::abs(len_target - 1)) {  // 停止したい距離の少し手前まで継続
    // 一定速度まで減速したら最低駆動トルクで走行

    if ((len_target >= 0 && tar_speed <= end_tar_speed) || (len_target < 0 && tar_speed >= end_tar_speed)) {  // 目標速度が最低速度になったら、加速度を0にする
      accel = 0;
      tar_speed = end_tar_speed;
    }

    HAL_Delay(1);
  }
  // 加速度を0にする
  accel = 0;
  tar_speed = (std::abs(end_speed) < FLT_EPSILON ? 0 : end_speed);

  if (is_enable_front_wall_control) {
    parts::wheel<bool, bool> is_front_wall_exsist = status.get_is_front_wall_control();
    if (is_front_wall_exsist.left && is_front_wall_exsist.right && std::abs(end_speed) < FLT_EPSILON) {
      side_wall_control = false;
      front_wall_control = true;
    }
    if (front_wall_control) {
      while (std::abs(status.get_speed()) > FLT_EPSILON || std::abs(status.get_ang_vel()) > std::abs(turn_vel_error)) {
        HAL_Delay(1);
      }
    }
    front_wall_control = false;
  }

  if (std::abs(end_speed) < FLT_EPSILON) {
    while (std::abs(status.get_speed()) > FLT_EPSILON) {
      HAL_Delay(1);
    }
  }
  if (std::abs(end_speed) < FLT_EPSILON) run_mode = parts::RunModeT::STOP_MODE;
  HAL_Delay(10);
  // 現在距離を0にリセット
  status.reset();
  if(std::abs(end_speed) < FLT_EPSILON) {
    speed.left.reset();
    speed.right.reset();
  }
  side_wall_control = side_wall_control_tmp;
  HAL_Delay(1);
}

template <typename T, class STATUS, class PID>
// positive: left, negative: right
void Controller<T, STATUS, PID>::turn(const float deg, float ang_accel, float max_ang_velocity) {
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
  ang_vel.reset();

  HAL_Delay(1);
}
}  // namespace state

#endif  // CORE_INC_CONTROLLER_HPP_
