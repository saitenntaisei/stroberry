#include "./gyro.hpp"
namespace spi {
Gyro::Gyro() : gyro_offset(0, 0, 0) {}
void Gyro::init() {
  spi_gyro_who_am_i();
  HAL_Delay(100);
  spi_gyro_write(0x20, 0x00);  // Power Down
  HAL_Delay(100);
  spi_gyro_write(0x20, 0b11111111);  // Normal Mode
  spi_gyro_write(0x21, 0b00010011);
  spi_gyro_write(0x24, 0b00010000);
  HAL_Delay(100);
  spi_gyro_write(0x23, 0b00100000);  // change sensitivity to 2000 dps
  HAL_Delay(100);
  geometry temp(0, 0, 0);
  const std::uint16_t times = 1000;
  for (int i = 0; i < times; i++) {  // calibration
    temp.x += spi_gyro_OUT_X();
    temp.y += spi_gyro_OUT_Y();
    temp.z += spi_gyro_OUT_Z();
    HAL_Delay(1);
  }
  gyro_offset.z = temp.z / times;
  gyro_offset.y = temp.y / times;
  gyro_offset.y = temp.y / times;
  printf("offset:=x:%f y:%f z:%f\r\n", gyro_offset.x, gyro_offset.y, gyro_offset.z);
}

void Gyro::spi_gyro_who_am_i(void) {
  HAL_Delay(100);
  while (true) {
    std::uint8_t report = spi_gyro_read(0x0F);
    printf("WHO_AM_I = %d\r\n", report);
    if (report == 215) break;
    HAL_Delay(100);
  }
}
float Gyro::spi_gyro_OUT_Z(void) {
  std::uint16_t Z_H = spi_gyro_read(0x2D);
  std::uint16_t Z_L = spi_gyro_read(0x2C);
  return static_cast<float>((std::int16_t)((Z_H << 8) + Z_L)) * gyro_sensitivty;
}
float Gyro::spi_gyro_OUT_X(void) {
  std::uint16_t X_H = spi_gyro_read(0x29);
  std::uint16_t X_L = spi_gyro_read(0x28);
  return static_cast<float>((std::int16_t)((X_H << 8) + X_L)) * gyro_sensitivty;
}
float Gyro::spi_gyro_OUT_Y(void) {
  std::uint16_t Y_H = spi_gyro_read(0x2B);
  std::uint16_t Y_L = spi_gyro_read(0x2A);
  return static_cast<float>((std::int16_t)((Y_H << 8) + Y_L)) * gyro_sensitivty;
}
geometry Gyro::read_gyro() {
  geometry dps(0, 0, 0);
  dps.z = spi_gyro_OUT_Z() - gyro_offset.z;
  dps.y = spi_gyro_OUT_Y() - gyro_offset.y;
  dps.x = spi_gyro_OUT_X() - gyro_offset.x;
  dps.z /= 1.027f;
  // if (abs(dps.z) < 0.5)
  //     dps.z = 0;
  // if (abs(dps.x) < 0.5)
  //     dps.x = 0;
  // if (abs(dps.y) < 0.5)
  //     dps.y = 0;

  return dps;
}

void Gyro::spi_gyro_write(std::uint8_t address, std::uint8_t value) {
  std::uint8_t transmit[2] = {address, value};
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin,
                    GPIO_PIN_RESET);  // CSピン立ち下げ
  HAL_Delay(1);
  HAL_SPI_Transmit(&hspi1, transmit, 2, 100);
  HAL_Delay(1);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin,
                    GPIO_PIN_SET);  // CSピン立ち上げ
}
std::uint8_t Gyro::spi_gyro_read(std::uint8_t address) {
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin,
                    GPIO_PIN_RESET);  // CSピン立ち下げ
  std::uint8_t transmit;
  transmit = address | 0x80;
  std::uint8_t receive = 0x00;
  HAL_SPI_Transmit(&hspi1, &transmit, 1, 100);
  HAL_SPI_Receive(&hspi1, &receive, 1, 100);
  // HAL_SPI_TransmitReceive(&hspi1, &transmit, &receive, 2, 100);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin,
                    GPIO_PIN_SET);  // CSピン立ち上げ
  return receive;
}
}  // namespace spi
