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
  static spi::Gyro gyro_;
  static parts::wheel<pwm::Encoder<float, std::int16_t>, pwm::Encoder<float, std::int16_t>> enc_;
  static parts::wheel<pwm::Motor, pwm::Motor> motor_;
  static state::Controller<float, state::Status<float>, state::Pid<float>> ctrl_;
  static adc::IrSensor<float> ir_sensor_;
  static adc::Battery<float, std::uint32_t> batt_;
  static pwm::IrLight ir_light_1_, ir_light_2_;
  static pwm::Buzzer buzzer_;

  static data::drive_records drive_rec_;
  static param::TestMode test_mode_;
  static float motor_signal_;
};

spi::Gyro GlobalState::gyro_;
state::Controller<float, state::Status<float>, state::Pid<float>> GlobalState::ctrl_;
adc::IrSensor<float> GlobalState::ir_sensor_(&hadc2, 4, 160, 10);
adc::Battery<float, std::uint32_t> GlobalState::batt_(&hadc1);
pwm::IrLight GlobalState::ir_light_1_(&htim9, TIM_CHANNEL_1), GlobalState::ir_light_2_(&htim9, TIM_CHANNEL_2);
pwm::Buzzer GlobalState::buzzer_(&htim12, TIM_CHANNEL_2);
data::drive_records GlobalState::drive_rec_;
param::TestMode GlobalState::test_mode_ = param::TestMode::NONE;
float GlobalState::motor_signal_ = 0.0f;
parts::wheel<pwm::Encoder<float, std::int16_t>, pwm::Encoder<float, std::int16_t>> GlobalState::enc_ = {pwm::Encoder<float, std::int16_t>(TIM1),
                                                                                                        pwm::Encoder<float, std::int16_t>(TIM8)};
parts::wheel<pwm::Motor, pwm::Motor> GlobalState::motor_ = {pwm::Motor(&htim4, &htim4, TIM_CHANNEL_3, TIM_CHANNEL_4), pwm::Motor(&htim4, &htim4, TIM_CHANNEL_1, TIM_CHANNEL_2)};
}  // namespace global_state

#endif  // CORE_INC_GLOBAL_STATE_HPP_
