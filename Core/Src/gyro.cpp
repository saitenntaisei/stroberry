#include "./gyro.hpp"

#include <array>

#include "./spi.h"
namespace spi {
Gyro::Gyro() : gyro_offset_(0, 0, 0) {}
void Gyro::Init() {
  SpiGyroWhoAmI();
  HAL_Delay(100);
  SpiGyroWrite(0x20, 0x00);  // Power Down
  HAL_Delay(100);
  SpiGyroWrite(0x20, 0b11111111);  // Normal Mode
  SpiGyroWrite(0x21, 0b00010011);
  SpiGyroWrite(0x24, 0b00010000);
  HAL_Delay(100);
  SpiGyroWrite(0x23, 0b00100000);  // change sensitivity to 2000 dps
  HAL_Delay(100);
  geometry temp(0, 0, 0);
  const std::uint16_t times = 1000;
  for (int i = 0; i < times; i++) {  // calibration
    temp.x += SpiGyroOutX();
    temp.y += SpiGyroOutY();
    temp.z += SpiGyroOutZ();
    HAL_Delay(1);
  }
  gyro_offset_.z = temp.z / times;
  gyro_offset_.y = temp.y / times;
  gyro_offset_.y = temp.y / times;
  printf("offset:=x:%f y:%f z:%f\r\n", gyro_offset_.x, gyro_offset_.y, gyro_offset_.z);
}

void Gyro::SpiGyroWhoAmI(void) {
  HAL_Delay(100);
  while (true) {
    std::uint8_t report = SpiGyroRead(0x0F);
    printf("WHO_AM_I = %d\r\n", report);
    if (report == 215) break;
    HAL_Delay(100);
  }
}
float Gyro::SpiGyroOutZ(void) {
  std::uint16_t Z_H = SpiGyroRead(0x2D);
  std::uint16_t Z_L = SpiGyroRead(0x2C);
  return static_cast<float>((std::int16_t)((Z_H << 8) + Z_L)) * kGyroSensitivity;
}
float Gyro::SpiGyroOutX(void) {
  std::uint16_t X_H = SpiGyroRead(0x29);
  std::uint16_t X_L = SpiGyroRead(0x28);
  return static_cast<float>((std::int16_t)((X_H << 8) + X_L)) * kGyroSensitivity;
}
float Gyro::SpiGyroOutY(void) {
  std::uint16_t Y_H = SpiGyroRead(0x2B);
  std::uint16_t Y_L = SpiGyroRead(0x2A);
  return static_cast<float>(static_cast<std::int16_t>((Y_H << 8) + Y_L)) * kGyroSensitivity;
}
geometry Gyro::ReadGyro() const {
  geometry dps(0, 0, 0);
  dps.z = SpiGyroOutZ() - gyro_offset_.z;
  dps.y = SpiGyroOutY() - gyro_offset_.y;
  dps.x = SpiGyroOutX() - gyro_offset_.x;
  dps.z /= 1.027f;

  return dps;
}

void Gyro::SpiGyroWrite(std::uint8_t address, std::uint8_t value) {
  std::array<std::uint8_t, 2> transmit_data = {address, value};
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);  // CSピン立ち下げ
  HAL_Delay(1);
  HAL_SPI_Transmit(&hspi1, transmit_data.data(), transmit_data.size(), 100);
  HAL_Delay(1);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin,
                    GPIO_PIN_SET);  // CSピン立ち上げ
}

std::uint8_t Gyro::SpiGyroRead(std::uint8_t address) {
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);  // CSピン立ち下げ
  std::uint8_t transmit = address | 0x80;
  std::uint8_t receive = 0x00;
  HAL_SPI_Transmit(&hspi1, &transmit, 1, 100);
  HAL_SPI_Receive(&hspi1, &receive, 1, 100);
  // HAL_SPI_TransmitReceive(&hspi1, &transmit, &receive, 2, 100);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);  // CSピン立ち上げ
  return receive;
}
}  // namespace spi
