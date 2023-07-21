#ifndef CORE_INC_IR_SENSOR_HPP_
#define CORE_INC_IR_SENSOR_HPP_

#include <cmath>
#include <memory>
#include <numbers>
#include <vector>

#include "./main.h"
#include "./mine.hpp"
#include "./motor.hpp"
namespace adc {

template <typename T>
class IrSensor {
 private:
  ADC_HandleTypeDef* hadc{};
  uint8_t ir_sensor_num = 0;
  std::unique_ptr<uint16_t[]> g_adc_data;
  std::unique_ptr<T[]> ir_sensor_value;
  std::unique_ptr<std::pair<float, float>[]> temp_ir_sensor_value;
  uint16_t counter_k = 0;
  uint16_t sampling_freq_kHz = 0;
  uint16_t ir_flashing_freq_kHz = 0;
  uint16_t sampling_times = 0;
  static constexpr uint16_t delta = 2000;
  std::vector<float> pre_cos, pre_sin;

 public:
  explicit IrSensor(ADC_HandleTypeDef* hadc, uint8_t num, uint16_t sampling_freq_kHz, uint16_t ir_flashing_freq_kHz);
  void ir_sampling(void);
  void ir_update(void);
  void ir_value_reset(void);
  T get_ir_value(uint8_t num) {
    if (num >= ir_sensor_num) {
      return -1;
    }

    return ir_sensor_value[num];
  }
};
template <typename T>
IrSensor<T>::IrSensor(ADC_HandleTypeDef* hadc, uint8_t num, uint16_t sampling_freq_kHz, uint16_t ir_flashing_freq_kHz)
    : hadc(hadc),
      ir_sensor_num(num),
      g_adc_data(new uint16_t[num]),
      ir_sensor_value(new T[num]),
      temp_ir_sensor_value(new std::pair<float, float>[num]),
      sampling_freq_kHz(sampling_freq_kHz),
      ir_flashing_freq_kHz(ir_flashing_freq_kHz),
      pre_cos(),
      pre_sin() {
  if (sampling_freq_kHz * 4 % ir_flashing_freq_kHz != 0) Error_Handler();
  sampling_times = static_cast<uint16_t>(sampling_freq_kHz * 4 / ir_flashing_freq_kHz);
  if (!HAL_ADC_Start_DMA(hadc, (uint32_t*)(g_adc_data.get()), ir_sensor_num) == HAL_OK) {
    Error_Handler();
  }
  const uint16_t furier_ratio = sampling_freq_kHz / ir_flashing_freq_kHz;
  for (uint8_t i = 0; i < sampling_times; i++) {
    pre_cos.push_back(std::cos(-2 * std::numbers::pi_v<float> * i / furier_ratio));
    pre_sin.push_back(std::sin(-2 * std::numbers::pi_v<float> * i / furier_ratio));
  }
  for (uint8_t i = 0; i < ir_sensor_num; i++) {
    ir_sensor_value[i] = 0;
    temp_ir_sensor_value[i] = std::make_pair(0, 0);
  }
}
template <typename T>
void IrSensor<T>::ir_sampling(void) {
  for (uint8_t i = 0; i < ir_sensor_num; i++) {
    temp_ir_sensor_value[i].first += (g_adc_data[i] - delta) * pre_cos[counter_k];
    temp_ir_sensor_value[i].second += (g_adc_data[i] - delta) * pre_sin[counter_k];
  }
  counter_k++;
  if (counter_k >= sampling_times) {
    ir_update();
  }
}
template <typename T>
void IrSensor<T>::ir_update(void) {
  for (uint8_t i = 0; i < ir_sensor_num; i++) {
    ir_sensor_value[i] = static_cast<T>(std::pow(temp_ir_sensor_value[i].first, 2) + std::pow(temp_ir_sensor_value[i].second, 2));
    temp_ir_sensor_value[i] = std::make_pair(0, 0);
  }

  counter_k = 0;
}
template <typename T>
void IrSensor<T>::ir_value_reset(void) {
  for (uint8_t i = 0; i < ir_sensor_num; i++) ir_sensor_value[i] = 0;
}

}  // namespace adc

namespace pwm {
class IrLight {
 private:
  timerPin ir_light;
  uint16_t freq;

 public:
  IrLight(TIM_HandleTypeDef* tim, unsigned int channel);
  void ir_flash_start();
  void ir_flash_stop();
};
IrLight::IrLight(TIM_HandleTypeDef* tim, unsigned int channel) : ir_light(tim, channel), freq(static_cast<uint16_t>(tim->Init.Period / 2)) {}
void IrLight::ir_flash_start() {
  HAL_TIM_PWM_Start(ir_light.tim, ir_light.channel);
  __HAL_TIM_SET_COMPARE(ir_light.tim, ir_light.channel, freq);  // Max 100
}
void IrLight::ir_flash_stop() {
  HAL_TIM_PWM_Stop(ir_light.tim, ir_light.channel);
  __HAL_TIM_SET_COMPARE(ir_light.tim, ir_light.channel, freq);  // Max 100
}

}  // namespace pwm
#endif  // CORE_INC_IR_SENSOR_HPP_