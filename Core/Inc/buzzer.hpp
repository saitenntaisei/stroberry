#ifndef CORE_INC_BUZZER_HPP_
#define CORE_INC_BUZZER_HPP_
#include <algorithm>
#include <string>

#include "motor.hpp"
namespace pwm {
class Buzzer {
 private:
  timerPin buzzer;
  static constexpr uint16_t freq = 500;

 public:
  Buzzer(TIM_HandleTypeDef* tim, unsigned int channel);
  void beep(std::string msg);
};
Buzzer::Buzzer(TIM_HandleTypeDef* tim, unsigned int channel) : buzzer(tim, channel) {}

void Buzzer::beep(std::string msg) {
  std::string morse[] = {
      ".-",    // A
      "-...",  // B
      "-.-.",  // C
      "-..",   // D
      ".",     // E
      "..-.",  // F
      "--.",   // G
      "....",  // H
      "..",    // I
      ".---",  // J
      "-.-",   // K
      ".-..",  // L
      "--",    // M
      "-.",    // N
      "---",   // O
      ".--.",  // P
      "--.-",  // Q
      ".-.",   // R
      "...",   // S
      "-",     // T
      "..-",   // U
      "...-",  // V
      ".--",   // W
      "-..-",  // X
      "-.--",  // Y
      "--.."   // Z
  };
  std::transform(msg.begin(), msg.end(), msg.begin(), ::toupper);
  for (auto c : msg) {
    std::string code = morse[(c - 'A')];
    if (c == ' ') {
      HAL_Delay(700);
      continue;
    }
    for (auto x : code) {
      if (x == '.') {
        __HAL_TIM_SET_COMPARE(buzzer.tim, buzzer.channel, freq);
        HAL_Delay(100);
        __HAL_TIM_SET_COMPARE(buzzer.tim, buzzer.channel, 0);
        HAL_Delay(200);
      } else if (x == '-') {
        __HAL_TIM_SET_COMPARE(buzzer.tim, buzzer.channel, freq);
        HAL_Delay(300);
        __HAL_TIM_SET_COMPARE(buzzer.tim, buzzer.channel, 0);
      }
      HAL_Delay(100);
    }
    HAL_Delay(300);
  }
}
}  // namespace pwm

#endif  // CORE_INC_BUZZER_HPP_
