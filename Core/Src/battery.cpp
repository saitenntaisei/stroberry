#include "battery.hpp"
namespace adc {
Battery::Battery(ADC_HandleTypeDef* hadc) : hadc(hadc) {}
float Battery::read_batt(void) {
  uint16_t adc_Value = 0;
  float adc_volt, batt_volt;
  uint16_t resolution = (1 << resolution_bit) - 1;
  HAL_ADC_Start(hadc);
  HAL_ADC_PollForConversion(&hadc1, 1000);
  adc_Value = HAL_ADC_GetValue(&hadc1);
  adc_volt = (float)adc_Value * 3.3 / resolution;
  // Voltage divider resistor Vbatt -> 20kΩ -> 10kΩ -> GND
  batt_volt = adc_volt * (20 + 10) / 10.0f;
  printf("adc_Value = %d, adc_volt = %.3f, batt_volt = %.3f\n\r", adc_Value,
         adc_volt, batt_volt);
  HAL_ADC_Stop(&hadc1);
  return batt_volt;
}
}  // namespace adc