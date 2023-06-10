#include "./motor.hpp"
namespace pwm {
Motor::Motor(TIM_HandleTypeDef* tim_1, TIM_HandleTypeDef* tim_2, unsigned int channel_1, unsigned int channel_2) : out_1(tim_1, channel_1), out_2(tim_2, channel_2) {}
void Motor::drive(int16_t duty) {
  // printf("duty: %d\r\n", duty);
  if (duty >= 1000) {
    duty = 999;
  } else if (duty <= -1000) {
    duty = -999;
  }
  if (duty >= 0) {
    __HAL_TIM_SET_COMPARE(out_1.tim, out_1.channel, abs(duty));
  } else if (duty < 0) {
    __HAL_TIM_SET_COMPARE(out_2.tim, out_2.channel, abs(duty));
  }
}
void Motor::brake() {
  HAL_TIM_PWM_Stop(out_1.tim, out_1.channel);
  HAL_TIM_PWM_Stop(out_2.tim, out_2.channel);
  HAL_TIM_PWM_Start(out_1.tim, out_1.tim->Init.Prescaler);
  HAL_TIM_PWM_Start(out_1.tim, out_1.tim->Init.Prescaler);
}
}  // namespace pwm
