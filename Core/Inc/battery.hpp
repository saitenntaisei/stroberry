#ifndef CORE_INC_BATTERY_HPP_
#define CORE_INC_BATTERY_HPP_

#include "./main.h"
#include "./mine.hpp"

namespace adc {
template <typename T, typename RESO>
class Battery {
 private:
  const float R0 = 10;
  const float R1 = 20;
  // R1<->R0<->GND
  const float V_ref = 3.3;
  const uint8_t resolution_bit = 12;
  const float threshold = 7.5;
  ADC_HandleTypeDef* hadc;

 public:
  explicit Battery(ADC_HandleTypeDef* hadc);
  T read_batt(void);
};

template <typename T, typename RESO>
Battery<T, RESO>::Battery(ADC_HandleTypeDef* hadc) : hadc(hadc) {}
template <typename T, typename RESO>
T Battery<T, RESO>::read_batt(void) {
  RESO adc_Value = 0;
  float adc_volt, batt_volt;
  RESO resolution = (1 << resolution_bit) - 1;
  HAL_ADC_Start(hadc);
  HAL_ADC_PollForConversion(&hadc1, 1000);
  adc_Value = HAL_ADC_GetValue(&hadc1);
  adc_volt = static_cast<float>(adc_Value) * 3.3 / resolution;
  // Voltage divider resistor Vbatt -> 20kΩ -> 10kΩ -> GND
  batt_volt = adc_volt * (20 + 10) / 10.0f;
  // printf("adc_Value = %d, adc_volt = %.3f, batt_volt = %.3f\n\r", adc_Value,
  //        adc_volt, batt_volt);

  HAL_ADC_Stop(&hadc1);
  if (batt_volt < threshold) {
    printf("no batt!\r\n");
    Error_Handler();  // no batt
  }
  return (T)batt_volt;
}

}  // namespace adc
#endif  // CORE_INC_BATTERY_HPP_
