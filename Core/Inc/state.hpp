#ifndef MY_STATE_HPP
#define MY_STATE_HPP
#include "gyro.hpp"
#include "mine.hpp"
namespace state {
template <typename T>
class Status {
 private:
  T right_speed_old, left_speed_old;
  T right_speed_new, left_speed_new;
  T previous_speed;
  const float diameter_wheel = 3.4;
  const float radius_wheel = diameter_wheel / 2.0;
  /* data */
 public:
  T speed, I_speed, D_speed;
  T left_speed, right_speed;

  T len_mouse;
  T ang_vel, I_ang_vel;
  Status(/* args */);
  template <class ENCODER>
  void update(ENCODER left_enc_fn, ENCODER right_enc_fn);
};
template <typename T>
Status<T>::Status() {}
template <typename T>
template <class ENCODER>
void Status<T>::update(ENCODER left_enc_fn,
                       ENCODER right_enc_fn) {  // unit is control freq(1ms)
  T left_rads = left_enc_fn();
  T right_rads = right_enc_fn();
  left_speed_new = left_rads * (T)radius_wheel;
  right_speed_new = right_rads * (T)diameter_wheel;
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
  D_speed = (previous_speed - speed);
  len_mouse += (left_speed_new + right_speed) / 2.0;
  // spi::geometry dps = gyro();
  // ang_vel = dps.z;
  // I_ang_vel += ang_vel;
  // if (I_ang_vel > 30 * 10000000000) {
  //   I_ang_vel = 30 * 10000000000;
  // } else if (I_ang_vel < -1 * 10000000000) {
  //   I_ang_vel = -1 * 10000000000;
  // }
}
}  // namespace state
#endif