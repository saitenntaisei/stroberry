#include "mine.hpp"
// Flashのsector1の先頭に配置される変数(ラベル)
// 配置と定義はリンカスクリプトで行う

extern "C" int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&huart4, (uint8_t *)ptr, len, 100);
  return len;
}

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

// Flashのsector1の内容を全てwork_ramに読み出す
// work_ramの先頭アドレスを返す
extern "C" uint8_t *Flash_load() {
  memcpy(work_ram, &_backup_flash_start, BACKUP_FLASH_SECTOR_SIZE);
  return work_ram;
}

// Flashのsector1を消去後、work_ramにあるデータを書き込む
extern "C" bool Flash_store() {
  // Flashをclear
  if (!Flash_clear()) return false;

  uint32_t *p_work_ram = (uint32_t *)work_ram;

  HAL_FLASH_Unlock();

  // work_ramにあるデータを4バイトごとまとめて書き込む
  HAL_StatusTypeDef result;
  const size_t write_cnt = BACKUP_FLASH_SECTOR_SIZE / sizeof(uint32_t);

  for (size_t i = 0; i < write_cnt; i++) {
    result = HAL_FLASH_Program(
        FLASH_TYPEPROGRAM_WORD,
        (uint32_t)(&_backup_flash_start) + sizeof(uint32_t) * i, p_work_ram[i]);
    if (result != HAL_OK) break;
  }

  HAL_FLASH_Lock();

  return result == HAL_OK;
}
