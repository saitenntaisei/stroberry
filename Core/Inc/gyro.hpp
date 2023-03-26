#ifndef MY_GYRO_HPP
#define MY_GYRO_HPP
#include "adc.h"
#include "gpio.h"
#include "main.h"
#include "mine.hpp"
#include "spi.h"
#include "tim.h"
#include "usart.h"
namespace spi {
typedef struct geometry {
  float x;
  float y;
  float z;
  geometry(float x, float y, float z) : x(x), y(y), z(z) {}
} geometry;

class Gyro {
 private:
  /* data */
  geometry gyro_offset;
  const float gyro_sensitivty = 0.070f;
  float spi_gyro_OUT_Z(void);
  float spi_gyro_OUT_X(void);
  float spi_gyro_OUT_Y(void);
  void spi_gyro_write(uint8_t address, uint8_t value);
  uint8_t spi_gyro_read(uint8_t address);

 public:
  Gyro();
  geometry read_gyro();
  void spi_gyro_who_am_i(void);
};

}  // namespace spi
#endif