#ifndef CORE_INC_ROBOT_OPERATION_HPP_
#define CORE_INC_ROBOT_OPERATION_HPP_

#include <plog/Log.h>

#include <cstdint>

#include "../lib/Mseq/Mseq.h"
#include "./global_state.hpp"
#include "flash.hpp"
#include "maze_run.hpp"

using global_state::GlobalState;

void maze_run::MazeRunner::RobotMove(const Direction &dir) {
  std::int8_t robot_dir_index = 0;
  while (1) {
    if (robot_dir_.byte == NORTH << robot_dir_index) break;
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
    if (is_start_block_) {
      GlobalState::batt_.monitoring_state_ = false;
      GlobalState::ctrl_.SetSideWallControl(false);
      GlobalState::ctrl_.Back1s();

      GlobalState::ctrl_.Reset();
      HAL_Delay(1);
      GlobalState::ctrl_.Straight(180.0 - 40.0, 200, 200, 200);
      GlobalState::batt_.monitoring_state_ = true;
      GlobalState::ctrl_.SetSideWallControl(true);

      is_start_block_ = false;
    } else {
      GlobalState::ctrl_.Straight(180.0, 200, 200, 200);
    }
  } else if (dir_diff == 1 || dir_diff == -3) {  // Right
    GlobalState::ctrl_.SetSideWallControl(false);
    GlobalState::ctrl_.Straight(90.0, 200, 200, 0.0);
    HAL_Delay(1);
    GlobalState::ctrl_.Turn(-90, 540, 720);
    HAL_Delay(1);
    GlobalState::ctrl_.Straight(90.0, 200, 200, 200);
    GlobalState::ctrl_.SetSideWallControl(true);

  } else if (dir_diff == -1 || dir_diff == 3) {  // LEFT
    GlobalState::ctrl_.SetSideWallControl(false);
    GlobalState::ctrl_.Straight(90.0, 200, 200, 0.0);
    HAL_Delay(1);
    GlobalState::ctrl_.Turn(90, 540, 720);
    HAL_Delay(1);
    GlobalState::ctrl_.Straight(90.0, 200, 200, 200);
    GlobalState::ctrl_.SetSideWallControl(true);
  } else {  // 180度ターン
    if (prev_wall_cnt_ == 3 || GlobalState::ctrl_.status_.GetFrontWall()) {
      GlobalState::ctrl_.Straight(90.0, 200, 200, 0.0);
      GlobalState::ctrl_.Turn(180, 540, 720);
      GlobalState::batt_.monitoring_state_ = false;
      GlobalState::ctrl_.SetSideWallControl(false);
      GlobalState::ctrl_.Back1s();

      GlobalState::ctrl_.Reset();
      HAL_Delay(1);
      GlobalState::ctrl_.Straight(180.0 - 40.0, 200, 200, 200);
      GlobalState::batt_.monitoring_state_ = true;
      GlobalState::ctrl_.SetSideWallControl(true);
    } else {
      GlobalState::ctrl_.Straight(90.0, 200, 200, 0.0);
      GlobalState::ctrl_.Turn(180, 540, 720);
      GlobalState::ctrl_.Straight(90.0, 200, 200, 200);
    }
  }

  robot_dir_ = dir;
  // robot positionをdirの分だけ動かす
  if (NORTH == dir.byte) {
    robot_position_ += IndexVec::vecNorth;
  } else if (SOUTH == dir.byte) {
    robot_position_ += IndexVec::vecSouth;
  } else if (EAST == dir.byte) {
    robot_position_ += IndexVec::vecEast;
  } else if (WEST == dir.byte) {
    robot_position_ += IndexVec::vecWest;
  }
  return;
}

void maze_run::MazeRunner::RobotStop() {
  GlobalState::ctrl_.Reset();
  return;
}

const Direction &maze_run::MazeRunner::GetWallData() {
  wall_ = 0;

  bool is_front_wall = GlobalState::ctrl_.status_.GetFrontWall();
  bool is_left_wall = GlobalState::ctrl_.status_.GetLeftWall();
  bool is_right_wall = GlobalState::ctrl_.status_.GetRightWall();
  HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, is_left_wall ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED6_GPIO_Port, LED5_Pin, is_front_wall ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED6_GPIO_Port, LED2_Pin, is_front_wall ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED6_GPIO_Port, LED1_Pin, is_right_wall ? GPIO_PIN_SET : GPIO_PIN_RESET);
  if (is_front_wall && !is_left_wall && !is_right_wall) {
    GlobalState::ctrl_.SetSideWallControl(false);
  } else {
    GlobalState::ctrl_.SetSideWallControl(true);
  }

  std::int8_t robot_dir_index = 0;
  while (1) {
    if (robot_dir_.byte == NORTH << robot_dir_index) break;
    robot_dir_index++;
  }

  if (is_front_wall) {
    wall_.byte |= robot_dir_;
  }

  if (is_right_wall) {
    wall_.byte |= NORTH << (robot_dir_index + 1) % 4;
  }

  if (is_left_wall) {
    if (robot_dir_index == 0)
      wall_.byte |= WEST;
    else
      wall_.byte |= NORTH << (robot_dir_index - 1) % 4;
  }

  prev_wall_cnt_ = wall_.nWall();

  return wall_;
}

namespace robot_operation {

void AdjustMode(std::uint8_t mode) {
  switch (mode) {
    case 1: {  // 360度回転テストモード - ジャイロセンサーの調整用
      HAL_TIM_Base_Start_IT(&htim10);
      HAL_TIM_Base_Start_IT(&htim11);
      HAL_TIM_Base_Start_IT(&htim13);
      GlobalState::ctrl_.Turn(3600, 540, 720);

      // GlobalState::ctrl.turn(-90, 540, 720);
      // GlobalState::ctrl.turn(-90, 540, 180);
    } break;
    case 2: {  // 直進テストモード - エンコーダー調整用
      HAL_TIM_Base_Start_IT(&htim10);
      HAL_TIM_Base_Start_IT(&htim11);
      HAL_TIM_Base_Start_IT(&htim13);
      // GlobalState::ctrl.set_side_wall_control(false);

      GlobalState::ctrl_.Back1s();
      GlobalState::ctrl_.Straight(180.0 * 8 - 40.0, 400, 800, 0.0);
    } break;
    case 3: {  // 旋回特性測定モード - M系列信号を使用したモーター応答測定
      Mseq mseq(7);
      HAL_TIM_Base_Start_IT(&htim10);
      HAL_TIM_Base_Start_IT(&htim6);
      HAL_TIM_Base_Start_IT(&htim11);
      HAL_Delay(1);
      GlobalState::test_mode_ = param::TestMode::TURN_MODE;
      for (int i = 0; i < (1 << 7) - 1; ++i) {
        std::uint8_t signal = mseq.update();
        GlobalState::motor_signal_ = (static_cast<float>(signal) - 0.5f) * 2 * 2.5f;
        GlobalState::motor_.left.DriveVcc(GlobalState::motor_signal_);
        GlobalState::motor_.right.DriveVcc(GlobalState::motor_signal_);
        HAL_Delay(400);
      }
      GlobalState::test_mode_ = param::TestMode::NONE;
      HAL_TIM_Base_Stop_IT(&htim10);
      HAL_TIM_Base_Stop_IT(&htim11);
      HAL_TIM_Base_Stop_IT(&htim6);
      GlobalState::motor_.left.DriveVcc(0);
      GlobalState::motor_.right.DriveVcc(0);

      PLOG(plog::info) << (GlobalState::drive_rec_.size() * sizeof(data::twist));
      if (GlobalState::drive_rec_.size() * sizeof(data::twist) > BACKUP_FLASH_SECTOR_SIZE) {
        HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
        Error_Handler();
      }

      std::copy(GlobalState::drive_rec_.begin(), GlobalState::drive_rec_.end(), reinterpret_cast<data::drive_record *>(flash::work_ram));

      flash::Store();
      GlobalState::buzzer_.Beep("save");

    } break;

    case 4: {  // 電圧を直接入力して直進
      Mseq mseq(6);
      HAL_TIM_Base_Start_IT(&htim10);
      HAL_TIM_Base_Start_IT(&htim6);
      HAL_TIM_Base_Start_IT(&htim11);
      HAL_Delay(1);
      GlobalState::test_mode_ = param::TestMode::STRAIGHT_MODE;
      // for (int i = 0; i < (1 << 6) - 1; ++i) {
      //   std::uint8_t signal = mseq.update();
      //   GlobalState::motor_signal = (static_cast<float>(signal) - 0.5f) * 2 * 2.5f;
      //   GlobalState::motor.left.drive_vcc(-GlobalState::motor_signal);
      //   GlobalState::motor.right.drive_vcc(GlobalState::motor_signal);
      //   HAL_Delay(400);
      // }
      GlobalState::motor_signal_ = 2.0f;
      GlobalState::motor_.left.DriveVcc(-GlobalState::motor_signal_);
      GlobalState::motor_.right.DriveVcc(GlobalState::motor_signal_ * 1.1f);
      HAL_Delay(2000);
      GlobalState::test_mode_ = param::TestMode::NONE;
      HAL_TIM_Base_Stop_IT(&htim10);
      HAL_TIM_Base_Stop_IT(&htim11);
      HAL_TIM_Base_Stop_IT(&htim6);
      GlobalState::motor_.left.DriveVcc(0);
      GlobalState::motor_.right.DriveVcc(0);
      PLOG(plog::info) << (GlobalState::drive_rec_.size() * sizeof(data::twist));
      if (GlobalState::drive_rec_.size() * sizeof(data::twist) > BACKUP_FLASH_SECTOR_SIZE) {
        HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
        Error_Handler();
      }

      std::copy(GlobalState::drive_rec_.begin(), GlobalState::drive_rec_.end(), reinterpret_cast<data::drive_record *>(flash::work_ram));

      flash::Store();
      GlobalState::buzzer_.Beep("save");

    } break;
    case 5: {  // フラッシュメモリから記録データを読み込み表示
      flash::Load();
      std::copy((data::drive_record *)flash::work_ram, (data::drive_record *)flash::work_ram + 4000, std::back_inserter(GlobalState::drive_rec_));
      PLOG(plog::info) << "speed, signal";
      for (auto rec : GlobalState::drive_rec_) {
        PLOG(plog::info) << rec.speed << ", " << rec.signal;

        HAL_Delay(1);
      }
      GlobalState::buzzer_.Beep("done");
    } break;
    case 6: {  // 赤外線センサーと壁検知状態のリアルタイム表示
      HAL_TIM_Base_Start_IT(&htim10);
      HAL_TIM_Base_Start_IT(&htim11);
      while (1) {
        float ir_value[4];

        for (std::uint8_t i = 0; i < 4; ++i) {
          ir_value[i] = GlobalState::ir_sensor_.GetIrValue(i);
        }

        PLOG(plog::info) << "FRONT_LEFT: " << ir_value[0] << ", FRONT_RIGHT: " << ir_value[1] << ", LEFT: " << ir_value[2] << ", RIGHT: " << ir_value[3]
                         << ", LEN: " << GlobalState::ctrl_.status_.GetLenMouse();
        HAL_Delay(1);
        HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GlobalState::ctrl_.status_.GetLeftWall() ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED6_GPIO_Port, LED5_Pin, GlobalState::ctrl_.status_.GetFrontWall() ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED6_GPIO_Port, LED2_Pin, GlobalState::ctrl_.status_.GetFrontWall() ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED6_GPIO_Port, LED1_Pin, GlobalState::ctrl_.status_.GetRightWall() ? GPIO_PIN_SET : GPIO_PIN_RESET);
      }

    } break;

    default:
      break;
  }
}

void TrueRunMode(std::uint8_t mode) {
  switch (mode) {
    case 0: {  // 　メインの迷路探索実行
      HAL_TIM_Base_Start_IT(&htim10);
      HAL_TIM_Base_Start_IT(&htim11);
      HAL_TIM_Base_Start_IT(&htim13);
      GlobalState::ctrl_.SetFrontWallControlPermission(true);
      // GlobalState::ctrl.set_side_wall_control(false);
      maze_run::MazeRunner runner;
      if (runner.SearchRun() != 0) {
        Error_Handler();
      }

      HAL_TIM_Base_Stop_IT(&htim10);
      HAL_TIM_Base_Stop_IT(&htim11);
      HAL_TIM_Base_Stop_IT(&htim13);

    } break;

    case 1: {
      if (!flash::Load()) {
        PLOG(plog::info) << "flash load error";
        Error_Handler();
      }
      char asciiData[MAZE_SIZE + 1][MAZE_SIZE + 1];
      std::memcpy(asciiData, flash::work_ram, sizeof(asciiData));
      Maze maze;
      maze.loadFromArray(asciiData);
      maze.printWall();
    } break;

    case 6: {  // 時計回り・反時計回りの回転テスト
      HAL_TIM_Base_Start_IT(&htim10);
      HAL_TIM_Base_Start_IT(&htim11);
      // GlobalState::ctrl.turn(3600, 540, 720);
      // GlobalState::ctrl.front_wall_control = true;

      GlobalState::ctrl_.Turn(3600, 540, 720);
      HAL_Delay(1000);
      GlobalState::ctrl_.Turn(-3600, 540, 720);
      // GlobalState::ctrl.turn(-90, 540, 180);
    } break;
    case 5: {  // 壁制御有効での直進
      HAL_TIM_Base_Start_IT(&htim10);
      HAL_TIM_Base_Start_IT(&htim11);
      GlobalState::ctrl_.SetSideWallControl(true);
      GlobalState::ctrl_.Back1s();
      GlobalState::ctrl_.Straight(5 * 180.0 - 40, 400, 800, 0.0);
    } break;

    case 7: {  // 壁センサーエラー値とセンサー値の詳細表示
      HAL_TIM_Base_Start_IT(&htim11);
      while (true) {
        parts::wheel<float, float> side_wall_sensor_error = GlobalState::ctrl_.status_.GetSideWallSensorError();
        parts::wheel<float, float> front_wall_sensor_error = GlobalState::ctrl_.status_.GetFrontWallSensorValue();
        PLOG(plog::info) << "front_left: " << front_wall_sensor_error.left << ", front_right: " << front_wall_sensor_error.right << ", left: " << side_wall_sensor_error.left
                         << ", right: " << side_wall_sensor_error.right;
        HAL_Delay(1);
        HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GlobalState::ctrl_.status_.GetLeftWall() ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED6_GPIO_Port, LED5_Pin, GlobalState::ctrl_.status_.GetFrontWall() ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED6_GPIO_Port, LED2_Pin, GlobalState::ctrl_.status_.GetFrontWall() ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED6_GPIO_Port, LED1_Pin, GlobalState::ctrl_.status_.GetRightWall() ? GPIO_PIN_SET : GPIO_PIN_RESET);
      }
    }
    default:
      break;
  }
}
}  // namespace robot_operation
#endif  // CORE_INC_ROBOT_OPERATION_HPP_
