#ifndef MY_PART_HPP
#define MY_PART_HPP
#include "adc.h"
#include "gpio.h"
#include "main.h"
#include "mine.hpp"
#include "spi.h"
#include "tim.h"
#include "usart.h"
namespace parts {
template <class LEFT, class RIGHT>
struct wheel {
  LEFT left;
  RIGHT right;
};

}  // namespace parts
#endif