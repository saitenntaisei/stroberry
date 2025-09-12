#ifndef CORE_INC_STATE_HPP_
#define CORE_INC_STATE_HPP_
#include <functional>

#include "parts.hpp"

namespace state {
template <typename T>
class Status {
 private:
  T right_speed_old_ = 0, left_speed_old_ = 0;
  T right_speed_new_ = 0, left_speed_new_ = 0;
  T previous_speed_ = 0;
  static constexpr float kDiameterWheel = 32.82F;
  static constexpr float kRadiusWheel = kDiameterWheel / 2.0F;
  T speed_ = 0;
  T left_speed_ = 0, right_speed_ = 0;
  T len_mouse_ = 0;
  T ts_ = 0;

  T ang_vel_ = 0;
  T degree_ = 0;
  bool front_wall_ = false;
  bool left_wall_ = false;
  bool right_wall_ = false;
  static constexpr parts::wheel<T, T> kSideWallControlTh = {12.0f, 12.0f};
  static constexpr parts::wheel<T, T> kFrontWallControlTh = {13.0f, 13.0f};
  parts::wheel<T, T> side_wall_sensor_error_ = {0, 0};
  parts::wheel<T, T> front_wall_sensor_error_ = {0, 0};
  parts::wheel<T, T> front_wall_sensor_value_ = {0, 0};
  static constexpr parts::wheel<T, T> kSideWallSensorRef = {12.8f, 12.8f};
  static constexpr parts::wheel<T, T> kFrontWallSensorRef = {14.5f, 14.5f};
  parts::wheel<bool, bool> is_side_wall_control_ = {false, false};
  parts::wheel<bool, bool> is_front_wall_control_ = {false, false};
  static constexpr float kLeftThreshold = 11.5f, kRightThreshold = 11.5f, kFrontThreshold = 24.0f;
  /* data */
 public:
  enum WallSensor { FRONT_LEFT, FRONT_RIGHT, LEFT, RIGHT };
  Status(T ts = 0.001F);  // NOLINT
  template <class LEFTENC, class RIGHTENC, T (LEFTENC::*LEFTENCFn)(), T (RIGHTENC::*RIGHTENCFn)()>
  void UpdateEncoder(LEFTENC &left_enc, RIGHTENC &right_enc);
  void UpdateGyro(std::function<T(void)> gyro_yaw);
  void UpdateWallSensor(std::function<float *(void)> wall_sensor);
  T GetAngVel() { return ang_vel_; }
  T GetAng() { return degree_; }
  T GetSpeed() { return speed_; }
  T GetLenMouse() { return len_mouse_; }
  void Reset() {
    len_mouse_ = 0;
    degree_ = 0;
  }
  bool GetFrontWall() { return front_wall_; }
  bool GetLeftWall() { return left_wall_; }
  bool GetRightWall() { return right_wall_; }
  parts::wheel<bool, bool> GetIsSideWallControl() { return is_side_wall_control_; }
  parts::wheel<bool, bool> GetIsFrontWallControl() { return is_front_wall_control_; }
  parts::wheel<T, T> GetSideWallSensorError() { return side_wall_sensor_error_; }
  parts::wheel<T, T> GetFrontWallSensorError() { return front_wall_sensor_error_; }
  parts::wheel<T, T> GetFrontWallSensorValue() { return front_wall_sensor_value_; }
};
template <typename T>
Status<T>::Status(T ts) : ts_(ts) {}
template <typename T>
template <class LEFTENC, class RIGHTENC, T (LEFTENC::*LEFTENCFn)(), T (RIGHTENC::*RIGHTENCFn)()>
void Status<T>::UpdateEncoder(LEFTENC &left_enc, RIGHTENC &right_enc) {  // unit is control freq(1ms)
  T left_rads = -(left_enc.*LEFTENCFn)();
  T right_rads = (right_enc.*RIGHTENCFn)();
  left_speed_new_ = left_rads * static_cast<T>(kRadiusWheel) * 100;    // mm/s
  right_speed_new_ = right_rads * static_cast<T>(kRadiusWheel) * 100;  // mm/s
  left_speed_old_ = left_speed_;
  right_speed_old_ = right_speed_;
  // lowpass
  // left_speed = left_speed_new * 0.1F + left_speed_old * 0.9F;     // NOLINT
  // right_speed = right_speed_new * 0.1F + right_speed_old * 0.9F;  // NOLINT
  left_speed_ = left_speed_new_;
  right_speed_ = right_speed_new_;
  previous_speed_ = speed_;
  speed_ = (left_speed_ + right_speed_) / 2;
  len_mouse_ += (left_speed_new_ + right_speed_new_) / 2 / 100;  // mm
}
template <typename T>
void Status<T>::UpdateWallSensor(std::function<float *(void)> wall_sensor) {
  float *wall_sensor_value = wall_sensor();

  if (static_cast<float>(wall_sensor_value[FRONT_LEFT]) > kFrontWallControlTh.left) {
    is_front_wall_control_.left = true;
    front_wall_sensor_error_.left = static_cast<float>(wall_sensor_value[FRONT_LEFT]) - kFrontWallSensorRef.left;
  } else {
    is_front_wall_control_.left = false;
    front_wall_sensor_error_.left = 0;
  }

  if (static_cast<float>(wall_sensor_value[FRONT_RIGHT]) > kFrontWallControlTh.right) {
    is_front_wall_control_.right = true;
    front_wall_sensor_error_.right = static_cast<float>(wall_sensor_value[FRONT_RIGHT]) - kFrontWallSensorRef.right;
  } else {
    is_front_wall_control_.right = false;
    front_wall_sensor_error_.right = 0;
  }
  if (wall_sensor_value[LEFT] > kLeftThreshold) {
    left_wall_ = true;
  } else {
    left_wall_ = false;
  }
  if (wall_sensor_value[RIGHT] > kRightThreshold) {
    right_wall_ = true;
  } else {
    right_wall_ = false;
  }
  if (static_cast<float>(wall_sensor_value[LEFT]) > kSideWallControlTh.left) {
    is_side_wall_control_.left = true;
    side_wall_sensor_error_.left = static_cast<float>(wall_sensor_value[LEFT]) - kSideWallSensorRef.left;
  } else {
    is_side_wall_control_.left = false;
    side_wall_sensor_error_.left = 0;
  }
  if (static_cast<float>(wall_sensor_value[RIGHT]) > kSideWallControlTh.right) {
    is_side_wall_control_.right = true;
    side_wall_sensor_error_.right = static_cast<float>(wall_sensor_value[RIGHT]) - kSideWallSensorRef.right;
  } else {
    is_side_wall_control_.right = false;
    side_wall_sensor_error_.right = 0;
  }

  if (wall_sensor_value[FRONT_LEFT] > kFrontThreshold / 2 && wall_sensor_value[FRONT_RIGHT] > kFrontThreshold / 2) {
    front_wall_ = true;
  } else {
    front_wall_ = false;
  }

  if ((right_wall_ == false || left_wall_ == false) && (wall_sensor_value[FRONT_LEFT] > kFrontThreshold / 2 || wall_sensor_value[FRONT_RIGHT] > kFrontThreshold / 2)) {
    front_wall_ = true;
  }
  front_wall_sensor_value_.left = static_cast<float>(wall_sensor_value[FRONT_LEFT]);
  front_wall_sensor_value_.right = static_cast<float>(wall_sensor_value[FRONT_RIGHT]);
}
template <typename T>
void Status<T>::UpdateGyro(std::function<T(void)> gyro_yaw) {  // unit is control freq(1ms)

  ang_vel_ = gyro_yaw();
  degree_ += ang_vel_ * ts_;
}
}  // namespace state
#endif  // CORE_INC_STATE_HPP_
