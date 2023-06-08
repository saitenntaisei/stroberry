#ifndef CORE_INC_ENCODER_HPP_
#define CORE_INC_ENCODER_HPP_
#include "./mine.hpp"
namespace pwm {
constexpr float deg2rad = std::numbers::pi_v<float> / 180.0f;

template <typename T, typename CNT>
class Encoder {
 private:
  const float gear_duty = 10;
  const uint8_t encoder_resolution = 12;
  CNT read_encoder_cnt(void);  // LSB
  T speed_rads;
  T cnt_total;
  TIM_TypeDef* tim{};

 public:
  explicit Encoder(TIM_TypeDef* tim);
  T read_encoder_value();  // dps and considered gear duty
  T get_incremental_degrees();
};
template <typename T, typename CNT>
Encoder<T, CNT>::Encoder(TIM_TypeDef* tim) : speed_rads(0), cnt_total(0), tim(tim) {
  // HAL_TIM_Encoder_Start(htim, tim_channel);
}

template <typename T, typename CNT>
CNT Encoder<T, CNT>::read_encoder_cnt(void) {
  CNT enc_buff = (CNT)tim->CNT;
  tim->CNT = 0;
  return static_cast<CNT>(enc_buff);
}
template <typename T, typename CNT>
T Encoder<T, CNT>::read_encoder_value() {  // return speed in rads
  T encoder_temp = static_cast<T>(read_encoder_cnt());
  encoder_temp *= 360;
  encoder_temp /= (gear_duty * encoder_resolution);
  cnt_total += encoder_temp;
  speed_rads = encoder_temp * deg2rad;  // * control_cycle_Hz;
  return speed_rads;
}

template <typename T, typename CNT>
T Encoder<T, CNT>::get_incremental_degrees() {
  return get_incremental_degrees;
}
}  // namespace pwm
#endif  // CORE_INC_ENCODER_HPP_
