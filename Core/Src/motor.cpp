#include "./motor.hpp"
namespace pwm {
Motor::Motor(TIM_HandleTypeDef* tim_1, TIM_HandleTypeDef* tim_2,
             unsigned int channel_1, unsigned int channel_2) {
  OUT_1 = {tim_1, channel_1};
  OUT_2 = {tim_2, channel_2};
}
void Motor::drive(int16_t duty) {
  if (duty > 0) {
    HAL_TIM_PWM_Stop(OUT_1.tim, OUT_1.channel);
    HAL_TIM_PWM_Stop(OUT_2.tim, OUT_2.channel);
    __HAL_TIM_SET_COMPARE(OUT_1.tim, OUT_1.channel, abs(duty));
    HAL_TIM_PWM_Start(OUT_1.tim, OUT_1.channel);
  } else if (duty < 0) {
    HAL_TIM_PWM_Stop(OUT_1.tim, OUT_1.channel);
    HAL_TIM_PWM_Stop(OUT_2.tim, OUT_2.channel);
    __HAL_TIM_SET_COMPARE(OUT_2.tim, OUT_2.channel, abs(duty));
    HAL_TIM_PWM_Start(OUT_2.tim, OUT_2.channel);
  } else {
    HAL_TIM_PWM_Stop(OUT_1.tim, OUT_1.channel);
    HAL_TIM_PWM_Stop(OUT_2.tim, OUT_2.channel);
  }
}
void Motor::brake() {
  HAL_TIM_PWM_Stop(OUT_1.tim, OUT_1.channel);
  HAL_TIM_PWM_Stop(OUT_2.tim, OUT_2.channel);
  HAL_TIM_PWM_Start(OUT_1.tim, OUT_1.tim->Init.Prescaler);
  HAL_TIM_PWM_Start(OUT_1.tim, OUT_1.tim->Init.Prescaler);
}
}  // namespace pwm
