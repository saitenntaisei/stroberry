#ifndef CORE_INC_STATE_HPP_
#define CORE_INC_STATE_HPP_
#include <functional>

namespace state {
template <typename T>
class Status {
 private:
  T right_speed_old = 0, left_speed_old = 0;
  T right_speed_new = 0, left_speed_new = 0;
  T previous_speed = 0, previous_ang_vel = 0;
  const float diameter_wheel = 32.0f;
  const float radius_wheel = diameter_wheel / 2.0f;
  /* data */
 public:
  T speed = 0, I_speed = 0, D_speed = 0;
  T left_speed = 0, right_speed = 0;
  T len_mouse = 0;

  T ang_vel = 0, I_ang_vel = 0, D_ang_vel = 0;
  T degree = 0;
  Status();
  template <class LEFTENC, class RIGHTENC, T (LEFTENC::*LEFTENCFn)(), T (RIGHTENC::*RIGHTENCFn)()>
  void update(LEFTENC &left_enc, RIGHTENC &right_enc, std::function<T(void)> gyro_yaw);
};
template <typename T>
Status<T>::Status() {}
template <typename T>
template <class LEFTENC, class RIGHTENC, T (LEFTENC::*LEFTENCFn)(), T (RIGHTENC::*RIGHTENCFn)()>
void Status<T>::update(LEFTENC &left_enc, RIGHTENC &right_enc,
                       std::function<T(void)> gyro_yaw) {  // unit is control freq(1ms)
  T left_rads = -(left_enc.*LEFTENCFn)();
  T right_rads = (right_enc.*RIGHTENCFn)();
  left_speed_new = left_rads * (T)radius_wheel * (T)100;
  right_speed_new = right_rads * (T)radius_wheel * (T)100;
  left_speed_old = left_speed;
  right_speed_old = right_speed;
  // lowpass
  left_speed = left_speed_new * 0.1 + left_speed_old * 0.9;
  right_speed = right_speed_new * 0.1 + right_speed_old * 0.9;
  previous_speed = speed;
  speed = (left_speed + right_speed) / 2.0;
  I_speed += speed;
  // I成分のオーバーフローとアンダーフロー対策
  if (I_speed > 30 * 10000000000) {
    I_speed = 30 * 10000000000;
  } else if (I_speed < -1 * 10000000000) {
    I_speed = -1 * 10000000000;
  }
  D_speed = (speed - previous_speed);
  len_mouse += (left_speed_new + right_speed) / 2.0;

  ang_vel = gyro_yaw();
  D_ang_vel = (ang_vel - previous_ang_vel);
  previous_ang_vel = ang_vel;
  I_ang_vel += ang_vel;
  degree += ang_vel * 0.001f;
  if (I_ang_vel > 30 * 10000000000) {
    I_ang_vel = 30 * 10000000000;
  } else if (I_ang_vel < -1 * 10000000000) {
    I_ang_vel = -1 * 10000000000;
  }
}
}  // namespace state
#endif  // CORE_INC_STATE_HPP_
