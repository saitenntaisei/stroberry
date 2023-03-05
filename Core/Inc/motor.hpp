#ifndef MY_MOTOR_HPP
#define MY_MOTOR_HPP
#include "adc.h"
#include "gpio.h"
#include "main.h"
#include "mine.hpp"
#include "spi.h"
#include "tim.h"
#include "usart.h"
namespace pwm {
typedef struct timer_pin {
  TIM_HandleTypeDef* tim;
  unsigned int channel;
} timer_pin;
class Motor {
 private:
  timer_pin OUT_1;
  timer_pin OUT_2;
  /* data */
 public:
  Motor(TIM_HandleTypeDef* tim_1, TIM_HandleTypeDef* tim_2,
        unsigned int channel_1, unsigned int channel_2);
  void drive(int16_t duty);
  void brake();
};

}  // namespace pwm
#endif