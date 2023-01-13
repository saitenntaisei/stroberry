#include "encoder.hpp"

namespace pwm {

Encoder::Encoder() : encoder(0, 0), cnt_total(0, 0) {
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
}
int32_t Encoder::read_left_encoder_value(void) {
  int32_t enc_buff = (int32_t)TIM2->CNT;
  TIM2->CNT = 0;
  return (int32_t)enc_buff;
}
int16_t Encoder::read_right_encoder_value(void) {
  int16_t enc_buff = (int16_t)TIM8->CNT;
  TIM8->CNT = 0;
  return (int16_t)enc_buff;
}
wheel Encoder::read_encoder_value(uint16_t control_cycle_Hz) {
  wheel encoder_temp(read_left_encoder_value(), read_right_encoder_value());
  encoder_temp.l *= 360;
  encoder_temp.r *= 360;
  encoder_temp.l /= gear_duty * encoder_resolution;
  encoder_temp.r /= gear_duty * encoder_resolution;
  cnt_total.l += encoder_temp.l;
  cnt_total.r += encoder_temp.r;
  encoder.l = encoder_temp.l * control_cycle_Hz / 360.0f;
  encoder.r = encoder_temp.r * control_cycle_Hz / 360.0f;
  return encoder;
}
}  // namespace pwm