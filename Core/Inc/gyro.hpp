#ifndef CORE_INC_GYRO_HPP_
#define CORE_INC_GYRO_HPP_
#include <cstdint>
#include <cstdio>
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
  static constexpr float gyro_sensitivty = 0.0702F;
  static float spi_gyro_OUT_Z(void);
  static float spi_gyro_OUT_X(void);
  static float spi_gyro_OUT_Y(void);
  static void spi_gyro_write(std::uint8_t address, std::uint8_t value);
  static std::uint8_t spi_gyro_read(std::uint8_t address);

 public:
  Gyro();
  [[nodiscard]] geometry read_gyro() const;
  void init();
  static void spi_gyro_who_am_i(void);
};

}  // namespace spi
#endif  // CORE_INC_GYRO_HPP_
