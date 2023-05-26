#ifndef CORE_INC_PID_HPP_
#define CORE_INC_PID_HPP_
#include <parts.hpp>
#include <queue>

namespace state {
template <typename T>
class Pid {
 private:
  T Kp, Ki, Kd;
  T error, error_prev, error_sum;
  T output;
  std::queue<T> que;

 public:
  explicit Pid(float Kp, float Ki, float Kd);
  T update(T target, T current);
};

template <typename T>
Pid<T>::Pid(float Kp, float Ki, float Kd) : Kp(Kp), Ki(Ki), Kd(Kd) {
  error = 0;
  error_prev = 0;
  error_sum = 0;
  output = 0;
  for (int i = 0; i < 5; i++) {
    que.push(0);
  }
}

template <typename T>
T Pid<T>::update(T target, T current) {
  error = target - current;
  error_sum += error;
  error_sum -= que.front();
  que.pop();
  que.push(error);
  if (error_sum > 30 * 10000000000) {
    error_sum = 30 * 10000000000;
  } else if (error_sum < -1 * 10000000000) {
    error_sum = -1 * 10000000000;
  }
  output = Kp * error + Ki * error_sum + Kd * (error - error_prev);
  error_prev = error;
  return output;
}

}  // namespace state

#endif  // CORE_INC_Pid_HPP_
