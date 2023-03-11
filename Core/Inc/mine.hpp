#ifndef MY_MINE_HPP
#define MY_MINE_HPP

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

extern char _backup_flash_start;
bool Flash_clear();
bool Flash_store(uint8_t *data, uint32_t size,
                 uint32_t address = (uint32_t)(&_backup_flash_start));
uint8_t *Flash_load(uint8_t *data, uint32_t size,
                    uint32_t address = (uint32_t)(&_backup_flash_start));
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif