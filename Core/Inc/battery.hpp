#ifndef CORE_INC_BATTERY_HPP_
#define CORE_INC_BATTERY_HPP_
#include <cstdint>
namespace adc {
template <typename T, typename RESO>
class Battery {
 private:
  static constexpr float kR0 = 10;
  static constexpr float kR1 = 20;
  // R1<->R0<->GND
  static constexpr float kVRef = 3.3F;
  static constexpr std::uint8_t kResolutionBits = 12;
  static constexpr float kThreshold = 6.5;
  ADC_HandleTypeDef* hadc_{};

 public:
  bool monitoring_state_ = true;
  explicit Battery(ADC_HandleTypeDef* hadc);
  T ReadBatt(void);
};

template <typename T, typename RESO>
Battery<T, RESO>::Battery(ADC_HandleTypeDef* hadc) : hadc_(hadc) {}
template <typename T, typename RESO>
T Battery<T, RESO>::ReadBatt(void) {
  RESO adc_value = 0;
  float adc_volt = 0;
  float batt_volt = 0;
  RESO resolution = (1 << kResolutionBits) - 1;
  __disable_irq();
  HAL_ADC_Start(hadc_);
  HAL_ADC_PollForConversion(&hadc1, 1);
  adc_value = HAL_ADC_GetValue(&hadc1);
  adc_volt = static_cast<float>(adc_value) * kVRef / static_cast<float>(resolution);
  // Voltage divider resistor Vbatt -> 20kΩ -> 10kΩ -> GND
  batt_volt = adc_volt * (kR1 + kR0) / kR0;
  // printf("adc_Value = %d, adc_volt = %.3f, batt_volt = %.3f\n\r", adc_Value,
  //        adc_volt, batt_volt);

  HAL_ADC_Stop(&hadc1);
  __enable_irq();
  if (batt_volt < kThreshold) {
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
    // if (monitoring_state) {
    //   printf("no batt!\r\n");  // NOLINT
    //   Error_Handler();         // no batt
    // }
  }
  return static_cast<T>(batt_volt);
}

}  // namespace adc
#endif  // CORE_INC_BATTERY_HPP_
