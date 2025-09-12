#ifndef CORE_INC_IR_SENSOR_HPP_
#define CORE_INC_IR_SENSOR_HPP_

#include <cmath>
#include <cstdint>
#include <memory>
#include <numbers>
#include <queue>
#include <utility>
#include <vector>

#include "motor.hpp"
namespace adc {

template <typename T>
class IrSensor {
 private:
  ADC_HandleTypeDef* hadc_{};
  std::uint8_t ir_sensor_count_ = 0;
  std::unique_ptr<std::uint16_t[]> adc_data_;

  std::list<std::unique_ptr<T[]>> ir_sensor_values_;
  std::unique_ptr<T[]> moving_average_;
  std::unique_ptr<std::pair<float, float>[]> temp_values_;
  std::uint16_t counter_k_ = 0;
  std::uint16_t sampling_freq_khz_ = 0;
  std::uint16_t ir_flashing_freq_khz_ = 0;
  std::uint16_t sampling_times_ = 0;
  static constexpr std::uint16_t kDelta = 2000;
  int moving_average_size_ = 3;
  std::vector<float> pre_cos_, pre_sin_;
  std::uint16_t temp_cnt_ = 0;

 public:
  explicit IrSensor(ADC_HandleTypeDef* hadc, std::uint8_t num, std::uint16_t sampling_freq_kHz, std::uint16_t ir_flashing_freq_kHz, int moving_average_num = 3);
  IrSensor(const IrSensor&) = delete;
  IrSensor& operator=(const IrSensor&) = delete;
  IrSensor(IrSensor&&) = delete;
  IrSensor& operator=(IrSensor&&) = delete;
  ~IrSensor() = default;
  void Init(void);
  void IrSampling(void);
  void IrUpdate(void);
  enum IrSelection { FRONT, SIDE };
  IrSelection ir_selection_ = FRONT;

  T GetIrValue(std::uint8_t num) {
    if (num >= ir_sensor_count_) {
      return -1;
    }

    return ir_sensor_values_.back()[num];
  }
  T* GetAverageIrValues(void) const { return moving_average_.get(); }
  T* GetIrValues(void) const { return ir_sensor_values_.back().get(); }
};
template <typename T>
IrSensor<T>::IrSensor(ADC_HandleTypeDef* hadc, std::uint8_t num, std::uint16_t sampling_freq_kHz, std::uint16_t ir_flashing_freq_kHz, int moving_average_num)
    : hadc_(hadc),
      ir_sensor_count_(num),
      adc_data_(new std::uint16_t[num]),
      ir_sensor_values_(),
      moving_average_(new T[num]),
      temp_values_(new std::pair<float, float>[num]),
      sampling_freq_khz_(sampling_freq_kHz),
      ir_flashing_freq_khz_(ir_flashing_freq_kHz),
      moving_average_size_(moving_average_num),
      pre_cos_(),
      pre_sin_() {
  for (std::uint8_t i = 0; i < ir_sensor_count_; i++) {
    moving_average_[i] = 0;
  }
}
template <typename T>
void IrSensor<T>::Init() {
  if (sampling_freq_khz_ * 4 % ir_flashing_freq_khz_ != 0) Error_Handler();
  sampling_times_ = static_cast<std::uint16_t>(sampling_freq_khz_ * 4 / ir_flashing_freq_khz_);
  if (!HAL_ADC_Start_DMA(hadc_, reinterpret_cast<uint32_t*>(adc_data_.get()), ir_sensor_count_) == HAL_OK) {
    Error_Handler();
  }
  const std::uint16_t furier_ratio = sampling_freq_khz_ / ir_flashing_freq_khz_;
  for (std::uint8_t i = 0; i < sampling_times_; i++) {
    pre_cos_.push_back(std::cos(-2 * std::numbers::pi_v<float> * i / furier_ratio));
    pre_sin_.push_back(std::sin(-2 * std::numbers::pi_v<float> * i / furier_ratio));
  }
  for (std::uint8_t i = 0; i < ir_sensor_count_; i++) {
    temp_values_[i] = std::make_pair(0, 0);
  }
  for (std::uint8_t i = 0; i < ir_sensor_count_; i++) {
    moving_average_[i] = 0;
  }
}
template <typename T>
void IrSensor<T>::IrSampling(void) {
  std::uint8_t ir_sensor_index = 0;
  if (ir_selection_ == SIDE) ir_sensor_index = 2;
  for (std::uint8_t i = ir_sensor_index; i < ir_sensor_index + 2; i++) {
    temp_values_[i].first += (adc_data_[i] - kDelta) * pre_cos_[counter_k_];
    temp_values_[i].second += (adc_data_[i] - kDelta) * pre_sin_[counter_k_];
  }
  counter_k_++;
  if (counter_k_ >= sampling_times_) {
    IrUpdate();
  }
}
template <typename T>
void IrSensor<T>::IrUpdate(void) {
  if (temp_cnt_ < 1) {
    temp_cnt_++;
    counter_k_ = 0;
    std::uint8_t ir_sensor_index = 0;
    if (ir_selection_ == SIDE) ir_sensor_index = 2;
    for (std::uint8_t i = ir_sensor_index; i < ir_sensor_index + 2; i++) {
      temp_values_[i] = std::make_pair(0, 0);
    }
    return;
  }
  if (ir_selection_ == FRONT) {
    ir_selection_ = SIDE;
    counter_k_ = 0;
    temp_cnt_ = 0;
    return;
  } else
    ir_selection_ = FRONT;
  std::unique_ptr<T[]> ir_sensor_value(new T[ir_sensor_count_]);
  for (std::uint8_t i = 0; i < ir_sensor_count_; i++) {
    ir_sensor_value[i] = static_cast<T>(std::sqrt(std::pow(temp_values_[i].first, 2) + std::pow(temp_values_[i].second, 2)));
    ir_sensor_value[i] = std::log2(ir_sensor_value[i]);
    temp_values_[i] = std::make_pair(0, 0);
  }

  ir_sensor_values_.push_front(std::move(ir_sensor_value));

  if (moving_average_size_ < static_cast<int>(ir_sensor_values_.size())) {
    ir_sensor_values_.pop_back();
  }

  for (std::uint8_t i = 0; i < ir_sensor_count_; i++) {
    moving_average_[i] = 0;
  }

  std::for_each(ir_sensor_values_.cbegin(), ir_sensor_values_.cend(), [&](const std::unique_ptr<T[]>& x) {
    for (std::uint8_t i = 0; i < ir_sensor_count_; i++) {
      moving_average_[i] = std::max(x[i], moving_average_[i]);
    }
  });

  counter_k_ = 0;
  temp_cnt_ = 0;
}

}  // namespace adc

namespace pwm {
class IrLight {
 private:
  timerPin ir_light_;
  std::uint16_t freq_ = 50;
  bool is_flash_ = false;

 public:
  IrLight(TIM_HandleTypeDef* tim, unsigned int channel);
  void IrFlashStart();
  void IrFlashStop();
};
IrLight::IrLight(TIM_HandleTypeDef* tim, unsigned int channel) : ir_light_(tim, channel) {}
void IrLight::IrFlashStart() {
  if (!is_flash_) {
    HAL_TIM_PWM_Start(ir_light_.tim, ir_light_.channel);
    is_flash_ = true;
  }
  __HAL_TIM_SET_COMPARE(ir_light_.tim, ir_light_.channel, freq_);  // Max 100
}
void IrLight::IrFlashStop() {
  // HAL_TIM_PWM_Stop(ir_light.tim, ir_light.channel);
  __HAL_TIM_SET_COMPARE(ir_light_.tim, ir_light_.channel, 0);  // Max 100
}

}  // namespace pwm
#endif  // CORE_INC_IR_SENSOR_HPP_
