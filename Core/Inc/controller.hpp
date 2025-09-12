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
  parts::wheel<PID, PID> speed_ = {PID(0.00809f, 0.031819f, 0.00048949f, 0.0f), PID(0.00809f, 0.031819f, 0.00048949f, 0.0f)},
                         front_wall_ = {PID(4.0f, 12.0000f, 0.2000f, 0.0f), PID(4.0f, 12.0000f, 0.2000f, 0.0f)},
                         ang_ = {PID(0.5f, 0.05f, 0.001f, 0.0f), PID(0.5f, 0.05f, 0.001f, 0.0f)};
  PID side_wall_ = PID(80.0f, 0.00f, 10.0f, 0.0f);
  PID ang_vel_ = PID(0.0121024f, 0.207247f, 0.0f, 0.0f);

  parts::wheel<T, T> motor_duty_ = {0, 0};
  T target_speed_ = 0, accel_ = 0;
  float target_ang_vel_ = 0, ang_acc_ = 0, target_degree_ = 0;
  float max_speed_ = 0, max_ang_vel_ = 0, max_degree_ = 0;
  static constexpr float kTurnMinVel = 36.0F;
  static constexpr float kTurnVelError = 0.1F;     // 3.0F;
  static constexpr float kLenStartDecVel = 10.0F;  // mm
  static constexpr float kMinSpeed = 100.0F;       // mm/s

  parts::RunModeT run_mode_ = parts::RunModeT::STOP_MODE;
  bool front_wall_control_ = false;
  bool side_wall_control_ = true;
  bool enable_front_wall_control_ = true;

 public:
  state::Status<float> status_;  // NOLINT

  Controller() : status_() {}
  void SetSideWallControl(bool side_wall_control) { this->side_wall_control_ = side_wall_control; }
  void SetFrontWallControlPermission(bool is_enable_front_wall_control) { this->enable_front_wall_control_ = is_enable_front_wall_control; }
  void Reset();
  void Update();
  template <class MOTOR, void (MOTOR::*DRIVEFn)(float)>
  void DriveMotor(MOTOR &left_motor, MOTOR &right_motor, const std::int8_t left_dir, const std::int8_t right_dir);
  void Back1s();
  void Turn(const float deg, float ang_accel, float max_ang_velocity);
  void Straight(T len, T acc, T max_sp, T end_sp);
  void GenerateTargetSpeed();
};

template <typename T, class STATUS, class PID>
void Controller<T, STATUS, PID>::Reset() {
  speed_.left.Reset();
  speed_.right.Reset();
  ang_vel_.Reset();
  ang_.left.Reset();
  ang_.right.Reset();
  side_wall_.Reset();
  front_wall_.left.Reset();
  front_wall_.right.Reset();
}
template <typename T, class STATUS, class PID>
void Controller<T, STATUS, PID>::Update() {
  GenerateTargetSpeed();
  motor_duty_.left = 0;
  motor_duty_.right = 0;

  if (run_mode_ == parts::RunModeT::STRAIGHT_MODE && !front_wall_control_) {
    if (side_wall_control_) {
      parts::wheel<T, T> side_wall_sensor_error = status_.GetSideWallSensorError();
      parts::wheel<bool, bool> is_side_wall = status_.GetIsSideWallControl();
      std::uint8_t n = 1;

      if (!is_side_wall.left || !is_side_wall.right) {
        n = 1;
      }
      if (!is_side_wall.left && !is_side_wall.right) n = 2;

      target_ang_vel_ += side_wall_.Update(0, side_wall_sensor_error.left - side_wall_sensor_error.right) * (float)n;
    }
  }

  motor_duty_.left += speed_.left.Update(target_speed_, status_.GetSpeed());
  motor_duty_.right += speed_.right.Update(target_speed_, status_.GetSpeed()) * 1.1f;
  float ang_vel_pid = ang_vel_.Update(target_ang_vel_, status_.GetAngVel());
  motor_duty_.left -= ang_vel_pid;
  motor_duty_.right += ang_vel_pid;

  if (front_wall_control_) {
    parts::wheel<T, T> front_wall_sensor_error = status_.GetFrontWallSensorError();
    motor_duty_.left += front_wall_.left.Update(0, front_wall_sensor_error.left);
    motor_duty_.right += front_wall_.right.Update(0, front_wall_sensor_error.right);
  }
  if (run_mode_ == parts::RunModeT::STRAIGHT_MODE) {
    target_ang_vel_ = 0;
  }
  if (run_mode_ == parts::RunModeT::STOP_MODE && !front_wall_control_) {
    motor_duty_.left -= ang_.left.Update(0.0F, status_.GetAng());
    motor_duty_.right += ang_.right.Update(0.0F, status_.GetAng());
  }
}

template <typename T, class STATUS, class PID>
template <class MOTOR, void (MOTOR::*DRIVEFn)(float)>
void Controller<T, STATUS, PID>::DriveMotor(MOTOR &left_motor, MOTOR &right_motor, const std::int8_t left_dir, const std::int8_t right_dir) {
  (left_motor.*DRIVEFn)(left_dir * static_cast<float>(motor_duty_.left));
  (right_motor.*DRIVEFn)(right_dir * static_cast<float>(motor_duty_.right));
}
template <typename T, class STATUS, class PID>
void Controller<T, STATUS, PID>::GenerateTargetSpeed() {
  // 直線の場合の目標速度生成
  if (run_mode_ == parts::RunModeT::STRAIGHT_MODE) {
    target_speed_ += accel_ * 0.001F;  // 目標速度を設定加速度で更新
    // 最高速度制限
    if (std::abs(target_speed_) > std::abs(max_speed_)) {
      target_speed_ = max_speed_;  // 目標速度を設定最高速度に設定
    }

  } else if (run_mode_ == parts::RunModeT::TURN_MODE) {
    // // 車体中心速度更新
    // tar_speed += accel * 0.001;
    // // 最高速度制限
    // if (tar_speed > max_speed) {
    //   tar_speed = max_speed;  // 目標速度を設定最高速度に設定
    // }

    // 角加速度更新
    target_ang_vel_ += ang_acc_ / 1000;  // 目標角速度を設定加速度で更新
    // tar_degree += tar_ang_vel / 1000;

    // 最高角速度制限
    if (std::abs(target_ang_vel_) > std::abs(max_ang_vel_)) {
      target_ang_vel_ = max_ang_vel_;  // 目標速度を設定最高速度に設定
    }
    // if (std::abs(tar_degree) > std::abs(max_degree)) {
    //   tar_degree = max_degree;
    // }
  }
}

template <typename T, class STATUS, class PID>
void Controller<T, STATUS, PID>::Back1s() {
  bool side_wall_control_tmp = side_wall_control_;
  side_wall_control_ = false;
  run_mode_ = parts::RunModeT::STRAIGHT_MODE;
  target_speed_ = -150;
  max_speed_ = -150;
  HAL_Delay(2000);
  target_speed_ = 0;
  max_speed_ = 0;
  HAL_Delay(500);
  status_.Reset();
  side_wall_control_ = side_wall_control_tmp;
  speed_.left.Reset();
  speed_.right.Reset();
}

template <typename T, class STATUS, class PID>
void Controller<T, STATUS, PID>::Straight(T len, T acc, T max_sp, T end_sp) {  // mm
  // 走行モードを直線にする
  run_mode_ = parts::RunModeT::STRAIGHT_MODE;
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
  accel_ = acc;
  // 最高速度を設定
  max_speed_ = max_sp;
  // 減速処理を始めるべき位置まで加速、定速区間を続行
  while ((len_target - kLenStartDecVel) - status_.GetLenMouse() /*mm*/ >
         (static_cast<float>(end_speed * end_speed) - static_cast<float>(target_speed_ * target_speed_)) / (static_cast<float>(-2 * accel_))) /*mm*/ {
    HAL_Delay(1);
  }
  // 減速処理開始
  bool side_wall_control_tmp = side_wall_control_;

  float end_tar_speed = (std::fabs(end_speed) < FLT_EPSILON ? kMinSpeed : end_speed);
  accel_ = -acc;
  while (std::abs(status_.GetLenMouse()) < std::abs(len_target - 1)) {  // 停止したい距離の少し手前まで継続
    // 一定速度まで減速したら最低駆動トルクで走行

    if ((len_target >= 0 && target_speed_ <= end_tar_speed) || (len_target < 0 && target_speed_ >= end_tar_speed)) {  // 目標速度が最低速度になったら、加速度を0にする
      accel_ = 0;
      target_speed_ = end_tar_speed;
    }

    HAL_Delay(1);
  }
  // 加速度を0にする
  accel_ = 0;
  target_speed_ = (std::abs(end_speed) < FLT_EPSILON ? 0 : end_speed);

  if (enable_front_wall_control_) {
    parts::wheel<bool, bool> is_front_wall_exsist = status_.GetIsFrontWallControl();
    if (is_front_wall_exsist.left && is_front_wall_exsist.right && std::abs(end_speed) < FLT_EPSILON) {
      side_wall_control_ = false;
      front_wall_control_ = true;
    }
    if (front_wall_control_) {
      Reset();
      HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
      HAL_Delay(500);
      while (std::abs(status_.GetSpeed()) > FLT_EPSILON || std::abs(status_.GetAngVel()) > std::abs(kTurnVelError)) {
        HAL_Delay(1);
      }
      HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
      Reset();
      status_.Reset();
    }
    front_wall_control_ = false;
  }

  if (std::abs(end_speed) < FLT_EPSILON) {
    while (std::abs(status_.GetSpeed()) > FLT_EPSILON) {
      HAL_Delay(1);
    }
  }
  if (std::abs(end_speed) < FLT_EPSILON) run_mode_ = parts::RunModeT::STOP_MODE;
  HAL_Delay(10);
  // 現在距離を0にリセット
  status_.Reset();
  if (std::abs(end_speed) < FLT_EPSILON) {
    Reset();
  }
  side_wall_control_ = side_wall_control_tmp;
  HAL_Delay(1);
}

template <typename T, class STATUS, class PID>
// positive: left, negative: right
void Controller<T, STATUS, PID>::Turn(const float deg, float ang_accel, float max_ang_velocity) {
  run_mode_ = parts::RunModeT::TURN_MODE;
  ang_accel = std::abs(ang_accel);
  max_ang_velocity = std::abs(max_ang_velocity);
  if (deg < 0) {
    ang_accel = -ang_accel;
    max_ang_velocity = -max_ang_velocity;
  }
  // tar_degree = 0;

  float local_degree = 0;
  accel_ = 0;
  target_speed_ = 0;
  target_ang_vel_ = 0;
  // 走行モードをスラロームモードにする

  // 車体の現在角度を取得
  local_degree = status_.GetAng();
  // tar_degree = 0;
  ang_acc_ = ang_accel;
  max_ang_vel_ = max_ang_velocity;
  max_degree_ = deg;
  // 角加速度、加速度、最高角速度設定
  while (std::abs(deg - (status_.GetAng() - local_degree)) > std::abs(target_ang_vel_ * target_ang_vel_ / (2 * ang_accel))) {
    HAL_Delay(1);
  }

  // BEEP();
  // 角減速区間に入るため、角加速度設定

  ang_acc_ = -ang_accel;

  while (std::abs(status_.GetAng() - local_degree) < std::abs(max_degree_)) {
    if (std::abs(target_ang_vel_) < kTurnMinVel) {
      ang_acc_ = 0;
      target_ang_vel_ = (target_ang_vel_ >= 0 ? kTurnMinVel : -kTurnMinVel);
    }
    HAL_Delay(1);
  }

  ang_acc_ = 0;
  target_ang_vel_ = 0;

  // tar_degree = max_degree;

  while (std::abs(status_.GetAngVel()) >= std::abs(kTurnVelError)) {  // NOLINT
    HAL_Delay(1);
  }

  target_ang_vel_ = 0;
  ang_acc_ = 0;
  // 現在距離を0にリセット
  status_.Reset();

  run_mode_ = parts::RunModeT::STOP_MODE;
  ang_vel_.Reset();

  HAL_Delay(1);
}
}  // namespace state

#endif  // CORE_INC_CONTROLLER_HPP_
