#include "mine.hpp"
#define BACKUP_FLASH_SECTOR_NUM FLASH_SECTOR_11
// Flashのsector1の先頭に配置される変数(ラベル)
// 配置と定義はリンカスクリプトで行う

extern "C" int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&huart4, (uint8_t *)ptr, len, 100);
  return len;
}

/*
 *@brief erase sector11
 */
extern "C" bool Flash_clear() {
  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef EraseInitStruct;
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;  // select sector
  EraseInitStruct.Sector =
      BACKUP_FLASH_SECTOR_NUM;    // set BACKUP_FLASH_SECTOR_NUM
  EraseInitStruct.NbSectors = 1;  // set to erase(EraseInitStruct) one sector
  EraseInitStruct.VoltageRange =
      FLASH_VOLTAGE_RANGE_3;  // set voltage range (2.7 to 3.6V)
  uint32_t error_sector;
  HAL_StatusTypeDef result = HAL_FLASHEx_Erase(&EraseInitStruct, &error_sector);
  printf("%lx\r\n", error_sector);
  HAL_FLASH_Lock();
  return result == HAL_OK && error_sector == 0xFFFFFFFF;
}

/*
 * @brief write flash(sector11)
 * @param uint32_t address sector11 start address
 * @param uint8_t * data write data
 * @param uint32_t size write data size
 */
extern "C" bool Flash_store(uint8_t *data, uint32_t size, uint32_t address) {
  // clear Flash
  if (!Flash_clear()) {
    printf("opps\n\r");
    return false;
  }

  HAL_FLASH_Unlock();

  HAL_StatusTypeDef result = HAL_OK;
  for (uint32_t add = address; add < (address + size); add++) {
    result =
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, add, *data);  // write byte
    data++;                                                     // add data
    if (result != HAL_OK) break;
  }

  HAL_FLASH_Lock();  // lock flash
  return result == HAL_OK;
}

/*
 * @brief write flash(sector11)
 * @param uint32_t address sector11 start address
 * @param uint8_t * data read data
 * @param uint32_t size read data size
 */
extern "C" uint8_t *Flash_load(uint8_t *data, uint32_t size, uint32_t address) {
  memcpy(data, (uint8_t *)address, size);  // copy data
  return data;
}