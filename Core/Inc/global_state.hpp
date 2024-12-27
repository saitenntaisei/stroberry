#ifndef CORE_INC_GLOBAL_STATE_HPP_
#define CORE_INC_GLOBAL_STATE_HPP_
#include <cstdint>

#include "./battery.hpp"
#include "./buzzer.hpp"
#include "./controller.hpp"
#include "./data.hpp"
#include "./encoder.hpp"
#include "./gyro.hpp"
#include "./ir_sensor.hpp"
#include "./motor.hpp"
#include "./param.hpp"
#include "./pid.hpp"
#include "./state.hpp"
namespace global_state {
class GlobalState {
 public:
  static spi::Gyro gyro;
  static parts::wheel<pwm::Encoder<float, std::int16_t>, pwm::Encoder<float, std::int16_t>> enc;
  static parts::wheel<pwm::Motor, pwm::Motor> motor;
  static state::Controller<float, state::Status<float>, state::Pid<float>> ctrl;
  static adc::IrSensor<float> ir_sensor;
  static adc::Battery<float, std::uint32_t> batt;
  static pwm::IrLight ir_light_1, ir_light_2;
  static pwm::Buzzer buzzer;

  static data::drive_records drive_rec;
  static param::TestMode test_mode;
  static float motor_signal;
};

spi::Gyro GlobalState::gyro;
state::Controller<float, state::Status<float>, state::Pid<float>> GlobalState::ctrl;
adc::IrSensor<float> GlobalState::ir_sensor(&hadc2, 4, 160, 10);
adc::Battery<float, std::uint32_t> GlobalState::batt(&hadc1);
pwm::IrLight GlobalState::ir_light_1(&htim9, TIM_CHANNEL_1), GlobalState::ir_light_2(&htim9, TIM_CHANNEL_2);
pwm::Buzzer GlobalState::buzzer(&htim12, TIM_CHANNEL_2);
data::drive_records GlobalState::drive_rec;
param::TestMode GlobalState::test_mode = param::TestMode::NONE;
float GlobalState::motor_signal = 0.0f;
parts::wheel<pwm::Encoder<float, std::int16_t>, pwm::Encoder<float, std::int16_t>> GlobalState::enc = {pwm::Encoder<float, std::int16_t>(TIM1),
                                                                                                       pwm::Encoder<float, std::int16_t>(TIM8)};
parts::wheel<pwm::Motor, pwm::Motor> GlobalState::motor = {pwm::Motor(&htim4, &htim4, TIM_CHANNEL_3, TIM_CHANNEL_4), pwm::Motor(&htim4, &htim4, TIM_CHANNEL_1, TIM_CHANNEL_2)};
}  // namespace global_state

#endif  // CORE_INC_GLOBAL_STATE_HPP_