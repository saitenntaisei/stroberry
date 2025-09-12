#include "./motor.hpp"
namespace pwm {
Motor::Motor(TIM_HandleTypeDef* tim_1, TIM_HandleTypeDef* tim_2, unsigned int channel_1, unsigned int channel_2) : out_1_(tim_1, channel_1), out_2_(tim_2, channel_2), batt_(&hadc1) {
  // HAL_TIM_PWM_Start(out_1.tim, out_1.channel);
  // HAL_TIM_PWM_Start(out_2.tim, out_2.channel);
}

void Motor::Init(void) {
  HAL_TIM_PWM_Start(out_1_.tim, out_1_.channel);
  HAL_TIM_PWM_Start(out_2_.tim, out_2_.channel);
}
void Motor::Drive(std::int16_t duty) {
  if (duty >= 100) {
    duty = 99;
  } else if (duty <= -100) {
    duty = -99;
  }
  if (duty >= 0) {
    __HAL_TIM_SET_COMPARE(out_1_.tim, out_1_.channel, abs(duty));
  } else if (duty < 0) {
    __HAL_TIM_SET_COMPARE(out_2_.tim, out_2_.channel, abs(duty));
  }
}
void Motor::DriveVcc(float volt) {
  float max_volt = batt_.ReadBatt();
  if (max_volt < volt) {
    volt = max_volt;
  } else if (volt < -max_volt) {
    volt = -max_volt;
  }
  if (volt >= 0) {
    __HAL_TIM_SET_COMPARE(out_1_.tim, out_1_.channel, static_cast<std::uint16_t>(std::abs(volt / max_volt * 100)));
    __HAL_TIM_SET_COMPARE(out_2_.tim, out_2_.channel, 0);
  } else if (volt < 0) {
    __HAL_TIM_SET_COMPARE(out_2_.tim, out_2_.channel, static_cast<std::uint16_t>(std::abs(volt / max_volt * 100)));
    __HAL_TIM_SET_COMPARE(out_1_.tim, out_1_.channel, 0);
  }
}
void Motor::Brake() {
  HAL_TIM_PWM_Stop(out_1_.tim, out_1_.channel);
  HAL_TIM_PWM_Stop(out_2_.tim, out_2_.channel);
  HAL_TIM_PWM_Start(out_1_.tim, out_1_.tim->Init.Prescaler);
  HAL_TIM_PWM_Start(out_1_.tim, out_1_.tim->Init.Prescaler);
}
}  // namespace pwm
