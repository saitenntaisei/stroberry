#ifndef CORE_INC_MOTOR_HPP_
#define CORE_INC_MOTOR_HPP_

#include <cmath>

#include "./tim.h"
#include "adc.h"
#include "battery.hpp"
namespace pwm {
using timer_pin = struct timerPin {
  TIM_HandleTypeDef* tim{};
  unsigned int channel{};
  timerPin(TIM_HandleTypeDef* tim, unsigned int channel) : tim(tim), channel(channel) {}
};
class Motor {
 private:
  timerPin out_1_;
  timerPin out_2_;
  adc::Battery<float, std::uint32_t> batt_;
  /* data */
 public:
  Motor(TIM_HandleTypeDef* tim_1, TIM_HandleTypeDef* tim_2, unsigned int channel_1, unsigned int channel_2);
  void Init(void);
  void Drive(int16_t duty);
  void DriveVcc(float volt);
  void Brake();
};

}  // namespace pwm
#endif  // CORE_INC_MOTOR_HPP_
