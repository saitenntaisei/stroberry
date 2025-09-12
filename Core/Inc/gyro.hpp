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
  geometry gyro_offset_;
  static constexpr float kGyroSensitivity = 0.0702F;
  static float SpiGyroOutZ(void);
  static float SpiGyroOutX(void);
  static float SpiGyroOutY(void);
  static void SpiGyroWrite(std::uint8_t address, std::uint8_t value);
  static std::uint8_t SpiGyroRead(std::uint8_t address);

 public:
  Gyro();
  [[nodiscard]] geometry ReadGyro() const;
  void Init();
  static void SpiGyroWhoAmI(void);
};

}  // namespace spi
#endif  // CORE_INC_GYRO_HPP_
