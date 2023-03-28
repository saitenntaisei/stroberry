#ifndef MY_MINE_HPP
#define MY_MINE_HPP
#define BACKUP_FLASH_SECTOR_NUM FLASH_SECTOR_1
#define BACKUP_FLASH_SECTOR_SIZE 1024 * 16

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#define PI 3.14159

#ifdef __cplusplus

extern "C" {
#endif /* __cplusplus */
#include <stdint.h>
#include <string.h>

#include "spi.h"
#include "usart.h"
// Flashから読みだしたデータを退避するRAM上の領域
// 4byteごとにアクセスをするので、アドレスが4の倍数になるように配置する
static uint8_t work_ram[BACKUP_FLASH_SECTOR_SIZE] __attribute__((aligned(4)));

// Flashのsector1の先頭に配置される変数(ラベル)
// 配置と定義はリンカスクリプトで行う
extern char _backup_flash_start;
bool Flash_clear();
bool Flash_store();
uint8_t* Flash_load();
#ifdef __cplusplus
}
#endif /* __cplusplus */
namespace text {
uint16_t Flash_string(std::string* str, uint16_t pos = 0);
template <typename... Args>
std::string format(const std::string& fmt, Args... args) {
  size_t len = std::snprintf(nullptr, 0, fmt.c_str(), args...);
  std::vector<char> buf(len + 1);
  std::snprintf(&buf[0], len + 1, fmt.c_str(), args...);
  return std::string(&buf[0], &buf[0] + len);
}
}  // namespace text

// std::string s = text::format("USSR %d\r\n", (uint16_t)1991);
// char *flash_data = (char *)Flash_load();
// printf("%s\r\n", flash_data);
// uint16_t pos = text::Flash_string(&s);
// s = text::format("Soviet %d\r\n", (uint16_t)1905);
// text::Flash_string(&s, pos);
// printf("%s\r\n", flash_data);
// if (!Flash_store()) {
//   printf("Failed to write flash\n");
// }
#endif
