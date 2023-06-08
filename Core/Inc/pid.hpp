#ifndef CORE_INC_PID_HPP_
#define CORE_INC_PID_HPP_
#include <queue>

#include "parts.hpp"

namespace state {
template <typename T>
class Pid {
 private:
  T kp, ki, kd;
  T ts;
  T error, error_prev, error_sum;
  T output;

  // std::queue<T> que;

 public:
  explicit Pid(T kp, T ki, T kd, T ts = 0.001F);  // NOLINT
  T update(T target, T current);
};

template <typename T>
Pid<T>::Pid(T kp, T ki, T kd, T ts) : kp(kp), ki(ki), kd(kd), ts(ts), error(0), error_prev(0), error_sum(0), output(0) {
  // for (int i = 0; i < 1000; i++) {
  //   que.push(0);
  // }
}

template <typename T>
T Pid<T>::update(T target, T current) {
  error = target - current;
  error_sum += error;
  // error_sum -= que.front();
  // que.pop();
  // que.push(error);
  output = kp * error + ki * error_sum * ts + kd * (error - error_prev) / ts;
  error_prev = error;
  return output;
}

}  // namespace state

#endif  // CORE_INC_Pid_HPP_
