#ifndef CORE_INC_MOTOR_HPP_
#define CORE_INC_MOTOR_HPP_

#include "./mine.hpp"
#include "./tim.h"

namespace pwm {
using timer_pin = struct timerPin {
  TIM_HandleTypeDef* tim{};
  unsigned int channel{};
  timerPin(TIM_HandleTypeDef* tim, unsigned int channel) : tim(tim), channel(channel) {}
};
class Motor {
 private:
  timerPin out_1;
  timerPin out_2;
  /* data */
 public:
  Motor(TIM_HandleTypeDef* tim_1, TIM_HandleTypeDef* tim_2, unsigned int channel_1, unsigned int channel_2);
  void drive(int16_t duty);
  void brake();
};

}  // namespace pwm
#endif  // CORE_INC_MOTOR_HPP_
