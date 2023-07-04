#ifndef CORE_INC_IR_SENSOR_HPP_
#define CORE_INC_IR_SENSOR_HPP_

#include <memory>

#include "./main.h"
#include "./mine.hpp"
#include "./motor.hpp"
namespace adc {

template <typename T>
class IrSensor {
 private:
  ADC_HandleTypeDef* hadc{};
  uint8_t ir_sensor_num = 0;
  std::unique_ptr<uint16_t[]> g_adc_data, temp_ir_sensor_value, ir_sensor_value;

 public:
  explicit IrSensor(ADC_HandleTypeDef* hadc, uint8_t num);
  void ir_sampling(void);
  void ir_update(void);
  void ir_value_reset(void);
  T get_ir_value(uint8_t num) {
    if (num >= ir_sensor_num || num < 0) {
      return -1;
    }
    return static_cast<T>(ir_sensor_value[num]);
  }
};
template <typename T>
IrSensor<T>::IrSensor(ADC_HandleTypeDef* hadc, uint8_t num)
    : hadc(hadc), ir_sensor_num(num), g_adc_data(new uint16_t[num]), temp_ir_sensor_value(new uint16_t[num]), ir_sensor_value(new uint16_t[num]) {
  static bool start_adc = [&]() {
    HAL_ADC_Start(hadc);
    return HAL_ADC_Start_DMA(&hadc2, (uint32_t*)(g_adc_data.get()), ir_sensor_num) == HAL_OK;
  }();
  if (!start_adc) {
    Error_Handler();
  }
}
template <typename T>
void IrSensor<T>::ir_sampling(void) {
  for (uint8_t i = 0; i < ir_sensor_num; i++) {
    if (g_adc_data[i] > ir_sensor_value[i]) ir_sensor_value[i] = g_adc_data[i];
  }
}
template <typename T>
void IrSensor<T>::ir_update(void) {
  for (uint8_t i = 0; i < ir_sensor_num; i++) {
    ir_sensor_value[i] = temp_ir_sensor_value[i];
    temp_ir_sensor_value[i] = 0;
  }
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
  static constexpr uint16_t freq = 500;

 public:
  IrLight(TIM_HandleTypeDef* tim, unsigned int channel);
  void ir_flash_start();
  void ir_flash_stop();
};
IrLight::IrLight(TIM_HandleTypeDef* tim, unsigned int channel) : ir_light(tim, channel) {}
void IrLight::ir_flash_start() {
  HAL_TIM_PWM_Start(ir_light.tim, ir_light.channel);
  __HAL_TIM_SET_COMPARE(ir_light.tim, ir_light.channel, 50);  // Max 100
}
void IrLight::ir_flash_stop() {
  HAL_TIM_PWM_Stop(ir_light.tim, ir_light.channel);
  __HAL_TIM_SET_COMPARE(ir_light.tim, ir_light.channel, 50);  // Max 100
}

}  // namespace pwm
#endif  // CORE_INC_IR_SENSOR_HPP_