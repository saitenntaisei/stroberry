#ifndef CORE_INC_PID_HPP_
#define CORE_INC_PID_HPP_
#include <queue>

#include "parts.hpp"

namespace state {
template <typename T>
class Pid {
 private:
  T kp, ki, kd, tf;
  T ts;
  T y_ = 0, y_prev_ = 0;
  T error_, error_prev_, error_sum_;
  T output_;

  // std::queue<T> que;

 public:
  explicit Pid(T kp, T ki, T kd, T tf, T ts = 0.001F);  // NOLINT
  T Update(T target, T current);
  void Reset();
};

template <typename T>
Pid<T>::Pid(T kp, T ki, T kd, T tf, T ts) : kp(kp), ki(ki), kd(kd), tf(tf), ts(ts), error_(0), error_prev_(0), error_sum_(0), output_(0) {
  // for (int i = 0; i < 1000; i++) {
  //   que.push(0);
  // }
}

template <typename T>
T Pid<T>::Update(T target, T current) {
  error_ = target - current;
  error_sum_ += error_;
  // error_sum -= que.front();
  // que.pop();
  // que.push(error);
  T a = ts / (ts + tf);
  T y = a * y_prev_ + (1 - a) * (error_ - error_prev_) / ts;
  output_ = kp * error_ + ki * error_sum_ * ts + kd * y;
  error_prev_ = error_;
  y_prev_ = y;
  return output_;
}
template <typename T>
void Pid<T>::Reset() {
  error_ = 0;
  error_prev_ = 0;
  error_sum_ = 0;
  y_ = 0;
  y_prev_ = 0;
}

}  // namespace state

#endif  // CORE_INC_PID_HPP_
