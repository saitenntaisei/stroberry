#ifndef CORE_INC_STATE_HPP_
#define CORE_INC_STATE_HPP_
#include <functional>

#include "parts.hpp"

namespace state {
template <typename T>
class Status {
 private:
  T right_speed_old = 0, left_speed_old = 0;
  T right_speed_new = 0, left_speed_new = 0;
  T previous_speed = 0;
  static constexpr float diameter_wheel = 32.8F;
  static constexpr float radius_wheel = diameter_wheel / 2.0F;
  T speed = 0;
  T left_speed = 0, right_speed = 0;
  T len_mouse = 0;
  T ts = 0;

  T ang_vel = 0;
  T degree = 0;
  bool front_wall = false;
  bool left_wall = false;
  bool right_wall = false;
  static constexpr parts::wheel<T, T> side_wall_control_th = {5000, 5000};
  static constexpr parts::wheel<T, T> front_wall_control_th = {6000, 6000};
  parts::wheel<T, T> side_wall_sensor_error = {0, 0};
  parts::wheel<T, T> front_wall_sensor_error = {0, 0};
  parts::wheel<T, T> front_wall_sensor_value = {0, 0};
  static constexpr parts::wheel<T, T> side_wall_sensor_ref = {9000, 7000};
  static constexpr parts::wheel<T, T> front_wall_sensor_ref = {28000, 28000};
  parts::wheel<bool, bool> is_side_wall_control = {false, false};
  parts::wheel<bool, bool> is_front_wall_control = {false, false};
  static constexpr std::uint32_t left_threshold = 3800, right_threshold = 3800, front_threshold = 8000;
  /* data */
 public:
  enum WallSensor { FRONT_LEFT, FRONT_RIGHT, LEFT, RIGHT };
  Status(T ts = 0.001F);  // NOLINT
  template <class LEFTENC, class RIGHTENC, T (LEFTENC::*LEFTENCFn)(), T (RIGHTENC::*RIGHTENCFn)()>
  void update_encoder(LEFTENC &left_enc, RIGHTENC &right_enc);
  void update_gyro(std::function<T(void)> gyro_yaw);
  void update_wall_sensor(std::function<std::uint32_t *(void)> wall_sensor);
  T get_ang_vel() { return ang_vel; }
  T get_ang() { return degree; }
  T get_speed() { return speed; }
  T get_len_mouse() { return len_mouse; }
  void reset() {
    len_mouse = 0;
    degree = 0;
  }
  bool get_front_wall() { return front_wall; }
  bool get_left_wall() { return left_wall; }
  bool get_right_wall() { return right_wall; }
  parts::wheel<bool, bool> get_is_side_wall_control() { return is_side_wall_control; }
  parts::wheel<bool, bool> get_is_front_wall_control() { return is_front_wall_control; }
  parts::wheel<T, T> get_side_wall_sensor_error() { return side_wall_sensor_error; }
  parts::wheel<T, T> get_front_wall_sensor_error() { return front_wall_sensor_error; }
  parts::wheel<T, T> get_front_wall_sensor_value() { return front_wall_sensor_value; }
};
template <typename T>
Status<T>::Status(T ts) : ts(ts) {}
template <typename T>
template <class LEFTENC, class RIGHTENC, T (LEFTENC::*LEFTENCFn)(), T (RIGHTENC::*RIGHTENCFn)()>
void Status<T>::update_encoder(LEFTENC &left_enc, RIGHTENC &right_enc) {  // unit is control freq(1ms)
  T left_rads = -(left_enc.*LEFTENCFn)();
  T right_rads = (right_enc.*RIGHTENCFn)();
  left_speed_new = left_rads * static_cast<T>(radius_wheel) * 100;    // mm/s
  right_speed_new = right_rads * static_cast<T>(radius_wheel) * 100;  // mm/s
  left_speed_old = left_speed;
  right_speed_old = right_speed;
  // lowpass
  // left_speed = left_speed_new * 0.1F + left_speed_old * 0.9F;     // NOLINT
  // right_speed = right_speed_new * 0.1F + right_speed_old * 0.9F;  // NOLINT
  left_speed = left_speed_new;
  right_speed = right_speed_new;
  previous_speed = speed;
  speed = (left_speed + right_speed) / 2;
  len_mouse += (left_speed_new + right_speed_new) / 2 / 100;  // mm
}
template <typename T>
void Status<T>::update_wall_sensor(std::function<std::uint32_t *(void)> wall_sensor) {
  std::uint32_t *wall_sensor_value = wall_sensor();

  if (static_cast<float>(wall_sensor_value[FRONT_LEFT]) > front_wall_control_th.left) {
    is_front_wall_control.left = true;
    front_wall_sensor_error.left = static_cast<float>(wall_sensor_value[FRONT_LEFT]) - front_wall_sensor_ref.left;
  } else {
    is_front_wall_control.left = false;
    front_wall_sensor_error.left = 0;
  }

  if (static_cast<float>(wall_sensor_value[FRONT_RIGHT]) > front_wall_control_th.right) {
    is_front_wall_control.right = true;
    front_wall_sensor_error.right = static_cast<float>(wall_sensor_value[FRONT_RIGHT]) - front_wall_sensor_ref.right;
  } else {
    is_front_wall_control.right = false;
    front_wall_sensor_error.right = 0;
  }
  if (wall_sensor_value[LEFT] > left_threshold) {
    left_wall = true;
  } else {
    left_wall = false;
  }
  if (wall_sensor_value[RIGHT] > right_threshold) {
    right_wall = true;
  } else {
    right_wall = false;
  }
  if (static_cast<float>(wall_sensor_value[LEFT]) > side_wall_control_th.left) {
    is_side_wall_control.left = true;
    side_wall_sensor_error.left = static_cast<float>(wall_sensor_value[LEFT]) - side_wall_sensor_ref.left;
  } else {
    is_side_wall_control.left = false;
    side_wall_sensor_error.left = 0;
  }
  if (static_cast<float>(wall_sensor_value[RIGHT]) > side_wall_control_th.right) {
    is_side_wall_control.right = true;
    side_wall_sensor_error.right = static_cast<float>(wall_sensor_value[RIGHT]) - side_wall_sensor_ref.right;
  } else {
    is_side_wall_control.right = false;
    side_wall_sensor_error.right = 0;
  }

  if (wall_sensor_value[FRONT_LEFT] > front_threshold / 2 && wall_sensor_value[FRONT_RIGHT] > front_threshold / 2) {
    front_wall = true;
  } else {
    front_wall = false;
  }

  if ((right_wall == false || left_wall == false) && (wall_sensor_value[FRONT_LEFT] > front_threshold / 2 || wall_sensor_value[FRONT_RIGHT] > front_threshold / 2)) {
    front_wall = true;
  }
  front_wall_sensor_value.left = static_cast<float>(wall_sensor_value[FRONT_LEFT]);
  front_wall_sensor_value.right = static_cast<float>(wall_sensor_value[FRONT_RIGHT]);
}
template <typename T>
void Status<T>::update_gyro(std::function<T(void)> gyro_yaw) {  // unit is control freq(1ms)

  ang_vel = gyro_yaw();
  degree += ang_vel * ts;
}
}  // namespace state
#endif  // CORE_INC_STATE_HPP_
