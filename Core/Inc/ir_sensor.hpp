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
  ADC_HandleTypeDef* hadc{};
  std::uint8_t ir_sensor_num = 0;
  std::unique_ptr<std::uint16_t[]> g_adc_data;

  std::queue<std::unique_ptr<T[]>> ir_sensor_values;
  std::unique_ptr<T[]> moving_average;
  std::unique_ptr<std::pair<float, float>[]> temp_ir_sensor_value;
  std::uint16_t counter_k = 0;
  std::uint16_t sampling_freq_kHz = 0;
  std::uint16_t ir_flashing_freq_kHz = 0;
  std::uint16_t sampling_times = 0;
  static constexpr std::uint16_t delta = 2000;
  int moving_average_num = 1;
  std::vector<float> pre_cos, pre_sin;

 public:
  explicit IrSensor(ADC_HandleTypeDef* hadc, std::uint8_t num, std::uint16_t sampling_freq_kHz, std::uint16_t ir_flashing_freq_kHz, int moving_average_num = 3);
  IrSensor(const IrSensor&) = delete;
  IrSensor& operator=(const IrSensor&) = delete;
  IrSensor(IrSensor&&) = delete;
  IrSensor& operator=(IrSensor&&) = delete;
  ~IrSensor() = default;
  void init(void);
  void ir_sampling(void);
  void ir_update(void);
  enum IrSelection { FRONT, SIDE };
  IrSelection ir_selection = FRONT;

  T get_ir_value(std::uint8_t num) {
    if (num >= ir_sensor_num) {
      return -1;
    }

    return ir_sensor_values.back()[num];
  }
  T* get_average_ir_values(void) const { return moving_average.get(); }
  T* get_ir_values(void) const { return ir_sensor_values.back().get(); }
};
template <typename T>
IrSensor<T>::IrSensor(ADC_HandleTypeDef* hadc, std::uint8_t num, std::uint16_t sampling_freq_kHz, std::uint16_t ir_flashing_freq_kHz, int moving_average_num)
    : hadc(hadc),
      ir_sensor_num(num),
      g_adc_data(new std::uint16_t[num]),
      ir_sensor_values(),
      moving_average(new T[num]),
      temp_ir_sensor_value(new std::pair<float, float>[num]),
      sampling_freq_kHz(sampling_freq_kHz),
      ir_flashing_freq_kHz(ir_flashing_freq_kHz),
      moving_average_num(moving_average_num),
      pre_cos(),
      pre_sin() {
  for (std::uint8_t i = 0; i < ir_sensor_num; i++) {
    moving_average[i] = 0;
  }
}
template <typename T>
void IrSensor<T>::init() {
  if (sampling_freq_kHz * 4 % ir_flashing_freq_kHz != 0) Error_Handler();
  sampling_times = static_cast<std::uint16_t>(sampling_freq_kHz * 4 / ir_flashing_freq_kHz);
  if (!HAL_ADC_Start_DMA(hadc, reinterpret_cast<uint32_t*>(g_adc_data.get()), ir_sensor_num) == HAL_OK) {
    Error_Handler();
  }
  const std::uint16_t furier_ratio = sampling_freq_kHz / ir_flashing_freq_kHz;
  for (std::uint8_t i = 0; i < sampling_times; i++) {
    pre_cos.push_back(std::cos(-2 * std::numbers::pi_v<float> * i / furier_ratio));
    pre_sin.push_back(std::sin(-2 * std::numbers::pi_v<float> * i / furier_ratio));
  }
  for (std::uint8_t i = 0; i < ir_sensor_num; i++) {
    temp_ir_sensor_value[i] = std::make_pair(0, 0);
  }
  for (std::uint8_t i = 0; i < ir_sensor_num; i++) {
    moving_average[i] = 0;
  }
}
template <typename T>
void IrSensor<T>::ir_sampling(void) {
  std::uint8_t ir_sensor_index = 0;
  if (ir_selection == SIDE) ir_sensor_index = 2;
  for (std::uint8_t i = ir_sensor_index; i < ir_sensor_index + 2; i++) {
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
  if (ir_selection == FRONT) {
    ir_selection = SIDE;
    counter_k = 0;
    return;
  } else
    ir_selection = FRONT;
  std::unique_ptr<T[]> ir_sensor_value(new T[ir_sensor_num]);
  for (std::uint8_t i = 0; i < ir_sensor_num; i++) {
    ir_sensor_value[i] = static_cast<T>(std::sqrt(std::pow(temp_ir_sensor_value[i].first, 2) + std::pow(temp_ir_sensor_value[i].second, 2)));
    temp_ir_sensor_value[i] = std::make_pair(0, 0);
  }

  for (std::uint8_t i = 0; i < ir_sensor_num; i++) {
    moving_average[i] += ir_sensor_value[i] / moving_average_num;
  }
  ir_sensor_values.push(std::move(ir_sensor_value));

  if (moving_average_num < static_cast<int>(ir_sensor_values.size())) {
    for (std::uint8_t i = 0; i < ir_sensor_num; i++) {
      moving_average[i] -= ir_sensor_values.front()[i] / moving_average_num;
    }
    ir_sensor_values.pop();
  }
  counter_k = 0;
}

}  // namespace adc

namespace pwm {
class IrLight {
 private:
  timerPin ir_light;
  std::uint16_t freq = 50;
  bool is_flash = false;

 public:
  IrLight(TIM_HandleTypeDef* tim, unsigned int channel);
  void ir_flash_start();
  void ir_flash_stop();
};
IrLight::IrLight(TIM_HandleTypeDef* tim, unsigned int channel) : ir_light(tim, channel) {}
void IrLight::ir_flash_start() {
  if (!is_flash) {
    HAL_TIM_PWM_Start(ir_light.tim, ir_light.channel);
    is_flash = true;
  }
  __HAL_TIM_SET_COMPARE(ir_light.tim, ir_light.channel, freq);  // Max 100
}
void IrLight::ir_flash_stop() {
  // HAL_TIM_PWM_Stop(ir_light.tim, ir_light.channel);
  __HAL_TIM_SET_COMPARE(ir_light.tim, ir_light.channel, 0);  // Max 100
}

}  // namespace pwm
#endif  // CORE_INC_IR_SENSOR_HPP_
