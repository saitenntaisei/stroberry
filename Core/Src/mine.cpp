#include "mine.hpp"
// Flashのsector1の先頭に配置される変数(ラベル)
// 配置と定義はリンカスクリプトで行う

// BEGINNOLINT
extern "C" int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&huart4, (uint8_t *)ptr, len, 100);  // NOLINT
  return len;
}
// ENDNOLINT

// BEGINNOLINT
// Flashのsectoe1を消去
extern "C" bool Flash_clear() {
  FLASH_WaitForLastOperation((uint32_t)50000U);
  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef EraseInitStruct;
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.Sector = BACKUP_FLASH_SECTOR_NUM;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.NbSectors = 1;

  // Eraseに失敗したsector番号がerror_sectorに入る
  // 正常にEraseができたときは0xFFFFFFFFが入る
  uint32_t error_sector = 12;
  HAL_StatusTypeDef result = HAL_FLASHEx_Erase(&EraseInitStruct, &error_sector);

  HAL_FLASH_Lock();

  return result == HAL_OK && error_sector == 0xFFFFFFFF;
}
// ENDNOLINT

// BEGINNOLINT
// Flashのsector1の内容を全てwork_ramに読み出す
// work_ramの先頭アドレスを返す
extern "C" uint8_t *Flash_load() {
  memcpy(work_ram, &_backup_flash_start, BACKUP_FLASH_SECTOR_SIZE);
  return work_ram;
}
// ENDNOLINT

// BEGINNOLINT
// Flashのsector1を消去後、work_ramにあるデータを書き込む
extern "C" bool Flash_store() {
  // Flashをclear
  if (!Flash_clear()) return false;

  uint32_t *p_work_ram = (uint32_t *)work_ram;  // NOLINT

  HAL_FLASH_Unlock();

  // work_ramにあるデータを4バイトごとまとめて書き込む
  HAL_StatusTypeDef result;
  const size_t write_cnt = BACKUP_FLASH_SECTOR_SIZE / sizeof(uint32_t);

  for (size_t i = 0; i < write_cnt; i++) {
    result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)(&_backup_flash_start) + sizeof(uint32_t) * i, p_work_ram[i]);
    if (result != HAL_OK) break;
  }

  HAL_FLASH_Lock();

  return result == HAL_OK;
}

/*
 * @brief write flash(sector11)
 * @param uint32_t address sector11 start address
 * @param uint8_t * data write data
 * @param uint32_t size write data size
 */
bool Flash_store_struct(uint8_t *data, uint32_t size) {
  if (!Flash_clear()) return false;  // erease sector1
  HAL_FLASH_Unlock();                // unlock flash
  uint32_t address = (uint32_t)(&_backup_flash_start);
  HAL_StatusTypeDef result = HAL_OK;
  for (uint32_t add = address; add < (address + size); add++, data++) {  // add data pointer
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, add, *data);               // write byte
    if (result != HAL_OK) break;
  }

  HAL_FLASH_Lock();  // lock flash
  return result == HAL_OK;
}
void Flash_load_struct(uint8_t *data, uint32_t size) {
  uint32_t address = (uint32_t)(&_backup_flash_start);
  memcpy(data, (uint8_t *)address, size);  // copy data
}
// ENDNOLINT
namespace text {
uint16_t Flash_string(std::string *str, uint16_t pos) {
  const char *cstr = str->c_str();
  if (strlen(cstr) + 1 > static_cast<size_t>(BACKUP_FLASH_SECTOR_SIZE - pos)) {
    return -1;
  }
  memcpy(work_ram + pos, cstr, strlen(cstr) + 1);  // sizeof(char) := 1
  Flash_store();
  return pos + strlen(cstr);
}
}  // namespace text
