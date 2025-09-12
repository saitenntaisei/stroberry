#ifndef CORE_INC_ENCODER_HPP_
#define CORE_INC_ENCODER_HPP_
#include <cstdint>
namespace pwm {
constexpr float kDegToRad = std::numbers::pi_v<float> / 180.0f;

template <typename T, typename CNT>
class Encoder {
 private:
  const float kGearDuty = 10;
  const std::uint8_t kEncoderResolution = 12;

  T speed_rads_;
  T cnt_total_;
  TIM_TypeDef* tim_{};
  CNT ReadEncoderCnt(void);  // LSB
 public:
  explicit Encoder(TIM_TypeDef* tim);
  T ReadEncoderValue();  // dps and considered gear duty
  T GetIncrementalDegrees();
};
template <typename T, typename CNT>
Encoder<T, CNT>::Encoder(TIM_TypeDef* tim) : speed_rads_(0), cnt_total_(0), tim_(tim) {
  // HAL_TIM_Encoder_Start(htim, tim_channel);
}

template <typename T, typename CNT>
CNT Encoder<T, CNT>::ReadEncoderCnt(void) {
  __disable_irq();
  CNT enc_buff = (CNT)tim_->CNT;
  tim_->CNT = 0;
  __enable_irq();

  return static_cast<CNT>(enc_buff);
}
template <typename T, typename CNT>
T Encoder<T, CNT>::ReadEncoderValue() {  // return speed in rads
  T encoder_temp = static_cast<T>(ReadEncoderCnt());
  encoder_temp *= 360;
  encoder_temp /= (kGearDuty * kEncoderResolution);
  cnt_total_ += encoder_temp;
  speed_rads_ = encoder_temp * kDegToRad;  // * control_cycle_Hz;
  return speed_rads_;
}

template <typename T, typename CNT>
T Encoder<T, CNT>::GetIncrementalDegrees() {
  return cnt_total_;
}
}  // namespace pwm
#endif  // CORE_INC_ENCODER_HPP_
