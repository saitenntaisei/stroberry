#ifndef MY_ENCODER_HPP
#define MY_ENCODER_HPP
#include "adc.h"
#include "gpio.h"
#include "main.h"
#include "mine.hpp"
#include "spi.h"
#include "tim.h"
#include "usart.h"
namespace pwm {

template <typename T, typename CNT>
class Encoder {
 private:
  const float gear_duty = 10;
  const uint8_t encoder_resolution = 12;
  CNT read_encoder_cnt(void);  // LSB
  TIM_TypeDef* tim;

 public:
  T speed_rads;
  T cnt_total;
  Encoder(TIM_TypeDef* tim);
  T read_encoder_value(
      uint16_t control_cycle_Hz = 1000);  // dps and considered gear duty
};
template <typename T, typename CNT>
Encoder<T, CNT>::Encoder(TIM_TypeDef* tim) : tim(tim) {
  speed_rads = 0;
  cnt_total = 0;
  // HAL_TIM_Encoder_Start(htim, tim_channel);
}

template <typename T, typename CNT>
CNT Encoder<T, CNT>::read_encoder_cnt(void) {
  CNT enc_buff = (CNT)tim->CNT;
  tim->CNT = 0;
  return (CNT)enc_buff;
}
template <typename T, typename CNT>
T Encoder<T, CNT>::read_encoder_value(uint16_t control_cycle_Hz) {
  T encoder_temp = (T)read_encoder_cnt();
  encoder_temp *= 360;
  encoder_temp /= (gear_duty * encoder_resolution);
  cnt_total += encoder_temp;
  speed_rads = encoder_temp * 2.0 * PI / 360;  // * control_cycle_Hz;
  return speed_rads;
}
}  // namespace pwm
#endif