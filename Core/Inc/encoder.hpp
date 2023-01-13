#ifndef MY_ENCODER_HPP
#define MY_ENCODER_HPP
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "mine.hpp"
namespace pwm
{
    typedef struct wheel
    {
        float l;
        float r;
        wheel(float l, float r) : l(l), r(r) {}
    } wheel;

    class Encoder
    {
    private:
        const float gear_duty = 10;
        const uint8_t encoder_resolution = 12;
        int32_t read_left_encoder_value(void);  // LSB
        int16_t read_right_encoder_value(void); // LSB

    public:
        wheel encoder;
        wheel cnt_total;
        Encoder();
        wheel read_encoder_value(uint16_t control_cycle_Hz = 1000); // dps and considered gear duty
    };

}
#endif