#ifndef MY_BATTERY_HPP
#define MY_BATTERY_HPP
#include "adc.h"
#include "gpio.h"
#include "main.h"
#include "mine.hpp"
#include "spi.h"
#include "tim.h"
#include "usart.h"
namespace adc {
class Battery {
 private:
  const float R0 = 10;
  const float R1 = 20;
  // R1<->R0<->GND
  const float V_ref = 3.3;
  const uint8_t resolution_bit = 12;
  ADC_HandleTypeDef* hadc;

 public:
  Battery(ADC_HandleTypeDef* hadc);
  float read_batt(void);
};

}  // namespace adc
#endif