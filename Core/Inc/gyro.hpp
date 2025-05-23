#ifndef CORE_INC_GYRO_HPP_
#define CORE_INC_GYRO_HPP_
#include <cstdint>
#include <cstdio>

#include "./spi.h"

namespace spi {
using geometry = struct geometry {
  float x;
  float y;
  float z;
  geometry(float x, float y, float z) : x(x), y(y), z(z) {}
};

class Gyro {
 private:
  /* data */
  geometry gyro_offset;
  const float gyro_sensitivty = 0.0702F;
  float spi_gyro_OUT_Z(void);
  float spi_gyro_OUT_X(void);
  float spi_gyro_OUT_Y(void);
  void spi_gyro_write(std::uint8_t address, std::uint8_t value);
  std::uint8_t spi_gyro_read(std::uint8_t address);

 public:
  Gyro();
  geometry read_gyro();
  void init();
  void spi_gyro_who_am_i(void);
};

}  // namespace spi
#endif  // CORE_INC_GYRO_HPP_
