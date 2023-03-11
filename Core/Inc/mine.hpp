#ifndef MY_MINE_HPP
#define MY_MINE_HPP
#define BACKUP_FLASH_SECTOR_NUM FLASH_SECTOR_1
#define BACKUP_FLASH_SECTOR_SIZE 1024 * 16

#include <memory>
#include <vector>

#include "gyro.hpp"

#define PI 3.14159

#ifdef __cplusplus

extern "C" {
#endif /* __cplusplus */
#include <stdint.h>
#include <string.h>

#include "adc.h"
#include "gpio.h"
#include "main.h"
#include "math.h"
#include "spi.h"
#include "stdarg.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"
#include "stm32f405xx.h"
#include "tim.h"
#include "usart.h"
// Flashから読みだしたデータを退避するRAM上の領域
// 4byteごとにアクセスをするので、アドレスが4の倍数になるように配置する
static uint8_t work_ram[BACKUP_FLASH_SECTOR_SIZE] __attribute__((aligned(4)));

// Flashのsector1の先頭に配置される変数(ラベル)
// 配置と定義はリンカスクリプトで行う
extern char _backup_flash_start;
bool Flash_clear();
bool Flash_store();
uint8_t *Flash_load();
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif