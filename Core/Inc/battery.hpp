#ifndef CORE_INC_BATTERY_HPP_
#define CORE_INC_BATTERY_HPP_

#include "./main.h"
#include "./mine.hpp"

namespace adc {
template <typename T, typename RESO>
class Battery {
 private:
  static constexpr float r0 = 10;
  static constexpr float r1 = 20;
  // R1<->R0<->GND
  static constexpr float v_ref = 3.3F;
  static constexpr uint8_t resolution_bit = 12;
  static constexpr float threshold = 7.0;
  ADC_HandleTypeDef* hadc{};

 public:
  explicit Battery(ADC_HandleTypeDef* hadc);
  T read_batt(void);
};

template <typename T, typename RESO>
Battery<T, RESO>::Battery(ADC_HandleTypeDef* hadc) : hadc(hadc) {}
template <typename T, typename RESO>
T Battery<T, RESO>::read_batt(void) {
  RESO adc_value = 0;
  float adc_volt = 0;
  float batt_volt = 0;
  RESO resolution = (1 << resolution_bit) - 1;
  HAL_ADC_Start(hadc);
  HAL_ADC_PollForConversion(&hadc1, 1000);
  adc_value = HAL_ADC_GetValue(&hadc1);
  adc_volt = static_cast<float>(adc_value) * v_ref / static_cast<float>(resolution);
  // Voltage divider resistor Vbatt -> 20kΩ -> 10kΩ -> GND
  batt_volt = adc_volt * (r1 + r0) / r0;
  // printf("adc_Value = %d, adc_volt = %.3f, batt_volt = %.3f\n\r", adc_Value,
  //        adc_volt, batt_volt);

  HAL_ADC_Stop(&hadc1);
  if (batt_volt < threshold) {
    printf("no batt!\r\n");  // NOLINT
    Error_Handler();         // no batt
  }
  return static_cast<T>(batt_volt);
}

}  // namespace adc
#endif  // CORE_INC_BATTERY_HPP_
