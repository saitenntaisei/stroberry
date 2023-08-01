#ifndef CORE_INC_STATE_HPP_
#define CORE_INC_STATE_HPP_
#include <functional>

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
  /* data */
 public:
  Status(T ts = 0.001F);  // NOLINT
  template <class LEFTENC, class RIGHTENC, T (LEFTENC::*LEFTENCFn)(), T (RIGHTENC::*RIGHTENCFn)()>
  void update_encoder(LEFTENC &left_enc, RIGHTENC &right_enc);
  void update_gyro(std::function<T(void)> gyro_yaw);
  T get_ang_vel() { return ang_vel; }
  T get_ang() { return degree; }
  T get_speed() { return speed; }
  T get_len_mouse() { return len_mouse; }
  void reset() {
    len_mouse = 0;
    degree = 0;
  }
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
  len_mouse += (left_speed_new + right_speed_new) / 2 * ts;  // mm
}
template <typename T>
void Status<T>::update_gyro(std::function<T(void)> gyro_yaw) {  // unit is control freq(1ms)

  ang_vel = gyro_yaw();
  degree += ang_vel * ts;
}
}  // namespace state
#endif  // CORE_INC_STATE_HPP_
