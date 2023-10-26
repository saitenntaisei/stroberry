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
  static constexpr float diameter_wheel = 32.0F;
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
  uint8_t wall_sensor_cnt = 0;
  static constexpr parts::wheel<T, T> control_th = {4000, 4000};
  parts::wheel<T, T> wall_sensor_error = {0, 0};
  static constexpr parts::wheel<T, T> wall_sensor_ref = {6500, 8000};
  parts::wheel<bool, bool> is_control = {false, false};
  static constexpr uint32_t left_threshold = 3500, right_threshold = 3500, front_threshold = 11000;
  /* data */
 public:
  enum WallSensor { FRONT_RIGHT, FRONT_LEFT, RIGHT, LEFT };
  Status(T ts = 0.001F);  // NOLINT
  template <class LEFTENC, class RIGHTENC, T (LEFTENC::*LEFTENCFn)(), T (RIGHTENC::*RIGHTENCFn)()>
  void update_encoder(LEFTENC &left_enc, RIGHTENC &right_enc);
  void update_gyro(std::function<T(void)> gyro_yaw);
  void update_wall_sensor(std::function<uint32_t *(void)> wall_sensor, std::function<void(void)> front_light, std::function<void(void)> side_light);
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
  parts::wheel<bool, bool> get_is_control() { return is_control; }
  parts::wheel<T, T> get_wall_sensor_error() { return wall_sensor_error; }
};
template <typename T>
Status<T>::Status(T ts) : ts(ts) {}
template <typename T>
template <class LEFTENC, class RIGHTENC, T (LEFTENC::*LEFTENCFn)(), T (RIGHTENC::*RIGHTENCFn)()>
void Status<T>::update_encoder(LEFTENC &left_enc, RIGHTENC &right_enc) {  // unit is control freq(1ms)
  T left_rads = (left_enc.*LEFTENCFn)();
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
void Status<T>::update_wall_sensor(std::function<uint32_t *(void)> wall_sensor, std::function<void(void)> front_light, std::function<void(void)> side_light) {
  uint32_t *wall_sensor_value = wall_sensor();
  switch (wall_sensor_cnt) {
    case 0:
      if (wall_sensor_value[FRONT_LEFT] + wall_sensor_value[FRONT_RIGHT] > front_threshold) {
        front_wall = true;
      } else {
        front_wall = false;
      }
      side_light();
      break;
    case 1:
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
      if (static_cast<float>(wall_sensor_value[LEFT]) > control_th.left) {
        is_control.left = true;
        wall_sensor_error.left = static_cast<float>(wall_sensor_value[LEFT]) - wall_sensor_ref.left;
      } else {
        is_control.left = false;
        wall_sensor_error.left = 0;
      }
      if (static_cast<float>(wall_sensor_value[RIGHT]) > control_th.right) {
        is_control.right = true;
        wall_sensor_error.right = static_cast<float>(wall_sensor_value[RIGHT]) - wall_sensor_ref.right;
      } else {
        is_control.right = false;
        wall_sensor_error.right = 0;
      }
      front_light();
      break;
    default:
      break;
  }
  wall_sensor_cnt++;
  wall_sensor_cnt %= 2;
}
template <typename T>
void Status<T>::update_gyro(std::function<T(void)> gyro_yaw) {  // unit is control freq(1ms)

  ang_vel = gyro_yaw();
  degree += ang_vel * ts;
}
}  // namespace state
#endif  // CORE_INC_STATE_HPP_
