#ifndef CORE_INC_MINE_HPP_
#define CORE_INC_MINE_HPP_
#define BACKUP_FLASH_SECTOR_NUM FLASH_SECTOR_1
#define BACKUP_FLASH_SECTOR_SIZE 1024 * 16
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "./spi.h"
#include "./usart.h"

extern char _backup_flash_start;
extern "C" int _write(int file, char* ptr, int len);

namespace param {
enum class TestMode : uint8_t { STRAIGHT_MODE, TURN_MODE, NONE };
}
namespace flash {
// Flashから読みだしたデータを退避するRAM上の領域
// 4byteごとにアクセスをするので、アドレスが4の倍数になるように配置する
inline uint8_t work_ram[BACKUP_FLASH_SECTOR_SIZE] __attribute__((aligned(4)));

// Flashのsector1の先頭に配置される変数(ラベル)
// 配置と定義はリンカスクリプトで行う

bool Clear();
bool Store();
uint8_t* Load();

bool Store_struct(uint8_t* data, uint32_t size);
void Load_struct(uint8_t* data, uint32_t size);
uint16_t Flash_string(std::string* str, uint16_t pos = 0);
}  // namespace flash
namespace text {

template <typename... Args>
std::string format(const std::string& fmt, Args... args) {
  size_t len = std::snprintf(nullptr, 0, fmt.c_str(), args...);
  std::vector<char> buf(len + 1);
  std::snprintf(&buf[0], len + 1, fmt.c_str(), args...);
  return std::string(&buf[0], &buf[0] + len);
}
}  // namespace text

// std::string s = text::format("USSR %d\r\n", (uint16_t)1991);
// char *flash_data = (char *)Load();
// printf("%s\r\n", flash_data);
// uint16_t pos = text::Flash_string(&s);
// s = text::format("Soviet %d\r\n", (uint16_t)1905);
// text::Flash_string(&s, pos);
// printf("%s\r\n", flash_data);
// if (!Store()) {
//   printf("Failed to write flash\n");
// }
#endif  // CORE_INC_MINE_HPP_
