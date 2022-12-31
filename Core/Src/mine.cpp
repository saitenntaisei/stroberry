#include "mine.hpp"
extern "C" int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart4, (uint8_t *)ptr, len, 100);
    return len;
}