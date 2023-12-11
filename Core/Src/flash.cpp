#include "flash.hpp"

// Flashのsector1の先頭に配置される変数(ラベル)
// 配置と定義はリンカスクリプトで行う

// BEGINNOLINT
extern "C" int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&huart4, (std::uint8_t *)ptr, len, 100);  // NOLINT
  return len;
}
// ENDNOLINT

namespace flash {

// Flashのsectoe1を消去
bool Clear() {
  FLASH_WaitForLastOperation((std::uint32_t)50000U);
  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef EraseInitStruct;
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.Sector = BACKUP_FLASH_SECTOR_NUM;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.NbSectors = 1;

  // Eraseに失敗したsector番号がerror_sectorに入る
  // 正常にEraseができたときは0xFFFFFFFFが入る
  std::uint32_t error_sector = 12;
  HAL_StatusTypeDef result = HAL_FLASHEx_Erase(&EraseInitStruct, &error_sector);

  HAL_FLASH_Lock();

  return result == HAL_OK && error_sector == 0xFFFFFFFF;
}

// Flashのsector1の内容を全てwork_ramに読み出す
// work_ramの先頭アドレスを返す
std::uint8_t *Load() {
  std::memcpy(work_ram, &_backup_flash_start, BACKUP_FLASH_SECTOR_SIZE);
  return work_ram;
}

// Flashのsector1を消去後、work_ramにあるデータを書き込む
bool Store() {
  // Flashをclear
  if (!Clear()) return false;

  std::uint32_t *p_work_ram = reinterpret_cast<std::uint32_t *>(work_ram);  // NOLINT

  HAL_FLASH_Unlock();

  // work_ramにあるデータを4バイトごとまとめて書き込む
  HAL_StatusTypeDef result;
  const size_t write_cnt = BACKUP_FLASH_SECTOR_SIZE / sizeof(std::uint32_t);

  for (size_t i = 0; i < write_cnt; i++) {
    result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, reinterpret_cast<std::uint32_t>(&_backup_flash_start) + sizeof(std::uint32_t) * i, p_work_ram[i]);
    if (result != HAL_OK) break;
  }

  HAL_FLASH_Lock();

  return result == HAL_OK;
}

/*
 * @brief write flash(sector11)
 * @param std::uint32_t address sector11 start address
 * @param std::uint8_t * data write data
 * @param std::uint32_t size write data size
 */
bool Store_struct(std::uint8_t *data, std::uint32_t size) {
  if (!Clear()) return false;  // erease sector1
  HAL_FLASH_Unlock();          // unlock flash
  std::uint32_t address = reinterpret_cast<std::uint32_t>(&_backup_flash_start);
  HAL_StatusTypeDef result = HAL_OK;
  for (std::uint32_t add = address; add < (address + size); add++, data++) {  // add data pointer
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, add, *data);                    // write byte
    if (result != HAL_OK) break;
  }

  HAL_FLASH_Lock();  // lock flash
  return result == HAL_OK;
}
void Load_struct(std::uint8_t *data, std::uint32_t size) {
  memcpy(data, &_backup_flash_start, size);  // copy data
}

std::uint16_t Flash_string(std::string *str, std::uint16_t pos) {
  const char *cstr = str->c_str();
  if (strlen(cstr) + 1 > static_cast<size_t>(BACKUP_FLASH_SECTOR_SIZE - pos)) {
    return -1;
  }
  memcpy(work_ram + pos, cstr, strlen(cstr) + 1);  // sizeof(char) := 1
  Store();
  return pos + strlen(cstr);
}
}  // namespace flash
