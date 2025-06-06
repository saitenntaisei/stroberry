#include "./motor.hpp"
namespace pwm {
Motor::Motor(TIM_HandleTypeDef* tim_1, TIM_HandleTypeDef* tim_2, unsigned int channel_1, unsigned int channel_2) : out_1(tim_1, channel_1), out_2(tim_2, channel_2), batt(&hadc1) {
  // HAL_TIM_PWM_Start(out_1.tim, out_1.channel);
  // HAL_TIM_PWM_Start(out_2.tim, out_2.channel);
}

void Motor::init(void) {
  HAL_TIM_PWM_Start(out_1.tim, out_1.channel);
  HAL_TIM_PWM_Start(out_2.tim, out_2.channel);
}
void Motor::drive(std::int16_t duty) {
  if (duty >= 100) {
    duty = 99;
  } else if (duty <= -100) {
    duty = -99;
  }
  if (duty >= 0) {
    __HAL_TIM_SET_COMPARE(out_1.tim, out_1.channel, abs(duty));
  } else if (duty < 0) {
    __HAL_TIM_SET_COMPARE(out_2.tim, out_2.channel, abs(duty));
  }
}
void Motor::drive_vcc(float volt) {
  float max_volt = batt.read_batt();
  if (max_volt < volt) {
    volt = max_volt;
  } else if (volt < -max_volt) {
    volt = -max_volt;
  }
  if (volt >= 0) {
    __HAL_TIM_SET_COMPARE(out_1.tim, out_1.channel, static_cast<std::uint16_t>(std::abs(volt / max_volt * 100)));
    __HAL_TIM_SET_COMPARE(out_2.tim, out_2.channel, 0);
  } else if (volt < 0) {
    __HAL_TIM_SET_COMPARE(out_2.tim, out_2.channel, static_cast<std::uint16_t>(std::abs(volt / max_volt * 100)));
    __HAL_TIM_SET_COMPARE(out_1.tim, out_1.channel, 0);
  }
}
void Motor::brake() {
  HAL_TIM_PWM_Stop(out_1.tim, out_1.channel);
  HAL_TIM_PWM_Stop(out_2.tim, out_2.channel);
  HAL_TIM_PWM_Start(out_1.tim, out_1.tim->Init.Prescaler);
  HAL_TIM_PWM_Start(out_1.tim, out_1.tim->Init.Prescaler);
}
}  // namespace pwm
