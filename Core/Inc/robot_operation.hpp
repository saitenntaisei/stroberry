#ifndef CORE_INC_ROBOTO_OPERATION_HPP_
#define CORE_INC_ROBOTO_OPERATION_HPP_

#include <cstdint>

#include "../lib/Mseq/Mseq.h"
#include "./global_state.hpp"
#include "flash.hpp"
#include "maze_run.hpp"

using global_state::GlobalState;

void maze_run::robot_move(Direction dir) {
  std::int8_t robot_dir_index = 0;
  while (1) {
    if (robot_dir.byte == NORTH << robot_dir_index) break;
    robot_dir_index++;
  }

  std::int8_t next_dir_index = 0;
  while (1) {
    if (dir.byte == NORTH << next_dir_index) break;
    next_dir_index++;
  }

  std::int8_t dir_diff = next_dir_index - robot_dir_index;
  // 直進
  if (dir_diff == 0) {
    if (is_start_block) {
      GlobalState::batt.monitoring_state = false;
      GlobalState::ctrl.back_1s();

      GlobalState::ctrl.reset();
      HAL_Delay(1);
      GlobalState::ctrl.straight(180.0 - 40.0, 200, 100, 100);
      GlobalState::batt.monitoring_state = true;

      is_start_block = false;
    } else {
      GlobalState::ctrl.straight(180.0, 200, 100, 100);
    }
  } else if (dir_diff == 1 || dir_diff == -3) {  // Right
    // GlobalState::ctrl.set_side_wall_control(false);
    GlobalState::ctrl.straight(90.0, 400, 800, 0.0);
    HAL_Delay(1);
    GlobalState::ctrl.turn(-90, 540, 720);
    HAL_Delay(1);
    GlobalState::ctrl.straight(90.0, 400, 800, 0.0);

  } else if (dir_diff == -1 || dir_diff == 3) {  // LEFT
    // GlobalState::ctrl.set_side_wall_control(false);
    GlobalState::ctrl.straight(90.0, 400, 800, 0.0);
    HAL_Delay(1);
    GlobalState::ctrl.turn(90, 540, 720);
    HAL_Delay(1);
    GlobalState::ctrl.straight(90.0, 400, 800, 0.0);
  } else {  // 180度ターン
    if (prev_wall_cnt == 3) {
      GlobalState::ctrl.straight(90.0, 400, 800, 0.0);
      GlobalState::ctrl.turn(180, 540, 720);
      GlobalState::batt.monitoring_state = false;
      GlobalState::ctrl.back_1s();

      GlobalState::ctrl.reset();
      HAL_Delay(1);
      GlobalState::ctrl.straight(180.0 - 40.0, 400, 800, 0.0);
      GlobalState::batt.monitoring_state = true;
    } else {
      GlobalState::ctrl.straight(90.0, 400, 800, 0.0);
      GlobalState::ctrl.turn(180, 540, 720);
      GlobalState::ctrl.straight(90.0, 400, 800, 0.0);
    }
  }

  robot_dir = dir;
  // robot positionをdirの分だけ動かす
  if (NORTH == dir.byte) {
    robot_position += IndexVec::vecNorth;
  } else if (SOUTH == dir.byte) {
    robot_position += IndexVec::vecSouth;
  } else if (EAST == dir.byte) {
    robot_position += IndexVec::vecEast;
  } else if (WEST == dir.byte) {
    robot_position += IndexVec::vecWest;
  }
  return;
}

Direction maze_run::get_wall_data() {
  Direction wall;
  std::uint8_t wall_front = 0;
  std::uint8_t wall_left = 0;
  std::uint8_t wall_right = 0;
  bool is_front_wall = GlobalState::ctrl.status.get_front_wall();
  bool is_left_wall = GlobalState::ctrl.status.get_left_wall();
  bool is_right_wall = GlobalState::ctrl.status.get_right_wall();
  HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, is_left_wall ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED6_GPIO_Port, LED5_Pin, is_front_wall ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED6_GPIO_Port, LED2_Pin, is_front_wall ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED6_GPIO_Port, LED1_Pin, is_right_wall ? GPIO_PIN_SET : GPIO_PIN_RESET);
  if (is_front_wall && !is_left_wall && !is_right_wall) {
    GlobalState::ctrl.set_side_wall_control(false);
  } else {
    GlobalState::ctrl.set_side_wall_control(true);
  }

  std::int8_t robot_dir_index = 0;
  while (1) {
    if (robot_dir.byte == NORTH << robot_dir_index) break;
    robot_dir_index++;
  }

  if (is_front_wall) {
    wall.byte |= robot_dir;
  }

  if (is_right_wall) {
    wall.byte |= NORTH << (robot_dir_index + 1) % 4;
  }

  if (is_left_wall) {
    if (robot_dir_index == 0)
      wall.byte |= WEST;
    else
      wall.byte |= NORTH << (robot_dir_index - 1) % 4;
  }

  prev_wall_cnt = wall.nWall();

  return wall;
}

namespace robot_operation {

void abjustMode(std::uint8_t mode) {
  switch (mode) {
    case 1: {
      HAL_TIM_Base_Start_IT(&htim10);
      HAL_TIM_Base_Start_IT(&htim11);
      // GlobalState::ctrl.turn(3600, 540, 720);

      GlobalState::ctrl.turn(3600, 540, 720);
      // GlobalState::ctrl.turn(-90, 540, 180);
    } break;
    case 2: {
      HAL_TIM_Base_Start_IT(&htim10);
      HAL_TIM_Base_Start_IT(&htim11);

      GlobalState::ctrl.back_1s();
      GlobalState::ctrl.straight(180.0 * 8 - 40.0, 400, 800, 0.0);
    } break;
    case 3: {
      Mseq mseq(7);
      HAL_TIM_Base_Start_IT(&htim10);
      HAL_TIM_Base_Start_IT(&htim6);
      HAL_TIM_Base_Start_IT(&htim11);
      HAL_Delay(1);
      GlobalState::test_mode = param::TestMode::TURN_MODE;
      for (int i = 0; i < (1 << 7) - 1; ++i) {
        std::uint8_t signal = mseq.update();
        GlobalState::motor_signal = (static_cast<float>(signal) - 0.5f) * 2 * 2.5f;
        GlobalState::motor.left.drive_vcc(GlobalState::motor_signal);
        GlobalState::motor.right.drive_vcc(GlobalState::motor_signal);
        HAL_Delay(400);
      }
      GlobalState::test_mode = param::TestMode::NONE;
      HAL_TIM_Base_Stop_IT(&htim10);
      HAL_TIM_Base_Stop_IT(&htim11);
      HAL_TIM_Base_Stop_IT(&htim6);
      GlobalState::motor.left.drive_vcc(0);
      GlobalState::motor.right.drive_vcc(0);

      printf("%d\r\n", GlobalState::drive_rec.size() * sizeof(data::twist));
      if (GlobalState::drive_rec.size() * sizeof(data::twist) > BACKUP_FLASH_SECTOR_SIZE) {
        HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
        Error_Handler();
      }

      std::copy(GlobalState::drive_rec.begin(), GlobalState::drive_rec.end(), reinterpret_cast<data::drive_record *>(flash::work_ram));

      flash::Store();
      GlobalState::buzzer.beep("save");

    } break;

    case 4: {
      Mseq mseq(6);
      HAL_TIM_Base_Start_IT(&htim10);
      HAL_TIM_Base_Start_IT(&htim6);
      HAL_TIM_Base_Start_IT(&htim11);
      HAL_Delay(1);
      GlobalState::test_mode = param::TestMode::STRAIGHT_MODE;
      // for (int i = 0; i < (1 << 6) - 1; ++i) {
      //   std::uint8_t signal = mseq.update();
      //   GlobalState::motor_signal = (static_cast<float>(signal) - 0.5f) * 2 * 2.5f;
      //   GlobalState::motor.left.drive_vcc(-GlobalState::motor_signal);
      //   GlobalState::motor.right.drive_vcc(GlobalState::motor_signal);
      //   HAL_Delay(400);
      // }
      GlobalState::motor_signal = 2.0f;
      GlobalState::motor.left.drive_vcc(-GlobalState::motor_signal);
      GlobalState::motor.right.drive_vcc(GlobalState::motor_signal * 1.1f);
      HAL_Delay(2000);
      GlobalState::test_mode = param::TestMode::NONE;
      HAL_TIM_Base_Stop_IT(&htim10);
      HAL_TIM_Base_Stop_IT(&htim11);
      HAL_TIM_Base_Stop_IT(&htim6);
      GlobalState::motor.left.drive_vcc(0);
      GlobalState::motor.right.drive_vcc(0);
      printf("%d\r\n", GlobalState::drive_rec.size() * sizeof(data::twist));
      if (GlobalState::drive_rec.size() * sizeof(data::twist) > BACKUP_FLASH_SECTOR_SIZE) {
        HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
        Error_Handler();
      }

      std::copy(GlobalState::drive_rec.begin(), GlobalState::drive_rec.end(), reinterpret_cast<data::drive_record *>(flash::work_ram));

      flash::Store();
      GlobalState::buzzer.beep("save");

    } break;
    case 5: {
      flash::Load();
      std::copy((data::drive_record *)flash::work_ram, (data::drive_record *)flash::work_ram + 4000, std::back_inserter(GlobalState::drive_rec));
      printf("speed, signal\r\n");
      for (auto rec : GlobalState::drive_rec) {
        printf("%f, %f\r\n", rec.speed, rec.signal);

        HAL_Delay(1);
      }
      GlobalState::buzzer.beep("done");
    } break;
    case 6: {
      // HAL_TIM_Base_Start_IT(&htim10);
      HAL_TIM_Base_Start_IT(&htim11);
      while (1) {
        std::uint32_t ir_value[4];
        GlobalState::ir_light_1.ir_flash_start();
        GlobalState::ir_light_2.ir_flash_stop();
        HAL_Delay(10);

        for (std::uint8_t i = 0; i < 2; ++i) {
          ir_value[i] = GlobalState::ir_sensor.get_ir_value(i);
        }
        GlobalState::ir_light_1.ir_flash_stop();
        GlobalState::ir_light_2.ir_flash_start();
        HAL_Delay(10);
        for (std::uint8_t i = 2; i < 4; ++i) {
          ir_value[i] = GlobalState::ir_sensor.get_ir_value(i);
        }

        printf("ir: %ld,%ld, %ld, %ld\r\n", ir_value[0], ir_value[1], ir_value[2], ir_value[3]);
        HAL_Delay(1);
        HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GlobalState::ctrl.status.get_left_wall() ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED6_GPIO_Port, LED5_Pin, GlobalState::ctrl.status.get_front_wall() ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED6_GPIO_Port, LED2_Pin, GlobalState::ctrl.status.get_front_wall() ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED6_GPIO_Port, LED1_Pin, GlobalState::ctrl.status.get_right_wall() ? GPIO_PIN_SET : GPIO_PIN_RESET);
      }

    } break;

    default:
      break;
  }
}

void trueRunMode(std::uint8_t mode) {
  switch (mode) {
    case 0: {
      HAL_TIM_Base_Start_IT(&htim10);
      HAL_TIM_Base_Start_IT(&htim11);

      // GlobalState::ctrl.set_side_wall_control(false);
      maze_run::search_run();

    } break;

    case 6: {
      HAL_TIM_Base_Start_IT(&htim10);
      HAL_TIM_Base_Start_IT(&htim11);
      // GlobalState::ctrl.turn(3600, 540, 720);
      // GlobalState::ctrl.front_wall_control = true;

      GlobalState::ctrl.turn(3600, 540, 720);
      HAL_Delay(1000);
      GlobalState::ctrl.turn(-3600, 540, 720);
      // GlobalState::ctrl.turn(-90, 540, 180);
    } break;
    case 5: {
      HAL_TIM_Base_Start_IT(&htim10);
      HAL_TIM_Base_Start_IT(&htim11);
      GlobalState::ctrl.set_side_wall_control(true);
      GlobalState::ctrl.back_1s();
      GlobalState::ctrl.straight(5 * 180.0 - 40, 400, 800, 0.0);
    } break;

    case 7: {
      HAL_TIM_Base_Start_IT(&htim11);
      while (true) {
        parts::wheel<float, float> side_wall_sensor_error = GlobalState::ctrl.status.get_side_wall_sensor_error();
        parts::wheel<float, float> front_wall_sensor_error = GlobalState::ctrl.status.get_front_wall_sensor_value();
        printf("front_left: %f, front_right: %f, left: %f, right: %f\r\n", front_wall_sensor_error.left, front_wall_sensor_error.right, side_wall_sensor_error.left,
               side_wall_sensor_error.right);
        HAL_Delay(1);
        HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GlobalState::ctrl.status.get_left_wall() ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED6_GPIO_Port, LED5_Pin, GlobalState::ctrl.status.get_front_wall() ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED6_GPIO_Port, LED2_Pin, GlobalState::ctrl.status.get_front_wall() ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED6_GPIO_Port, LED1_Pin, GlobalState::ctrl.status.get_right_wall() ? GPIO_PIN_SET : GPIO_PIN_RESET);
      }
    }
    default:
      break;
  }
}
}  // namespace robot_operation
#endif