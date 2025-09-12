#include "../Inc/maze_run.hpp"

#include <cstdio>
#include <cstring>

#include "../../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"
#include "../Inc/flash.hpp"
#include "../lib/MazeSolver2015/MazeSolver_conf.h"
#include <plog/Log.h>

namespace maze_run {
// メンバ関数の実装
int MazeRunner::SearchRun() {
  auto save_maze_to_flash = [this]() -> int {
    char asciiData[MAZE_SIZE + 1][MAZE_SIZE + 1];
    maze_.saveToArray(asciiData);
    std::memcpy(flash::work_ram, asciiData, sizeof(asciiData));
    if (!flash::Store()) {
      PLOG(plog::info) << "flash store error";
      return -1;
    }
    return 0;
  };

  while (true) {
    // センサから取得した壁情報を入れる
    const Direction kWallData = GetWallData();
    // ロボットの座標を取得
    const IndexVec kRobotPos = GetRobotPosition();

    // 壁情報を更新 次に進むべき方向を計算
    agent_.update(kRobotPos, kWallData);
    // Agentの状態を確認
    // FINISHEDになったら計測走行にうつる
    if (agent_.getState() == Agent::FINISHED) {
      if (save_maze_to_flash() != 0) {
        return -1;
      }
      break;
    }

    // ゴールにたどり着いた瞬間に一度だけmazeのバックアップをとる
    // Mazeクラスはoperator=が定義してあるからa = bでコピーできる
    if (prev_state_ == Agent::SEARCHING_NOT_GOAL && agent_.getState() != Agent::SEARCHING_NOT_GOAL) {
      if (save_maze_to_flash() != 0) {
        return -1;
      }
    }
    prev_state_ = agent_.getState();

    // Agentの状態が探索中の場合は次に進むべき方向を取得する
    const Direction& next_dir = agent_.getNextDirection();

    // next_dirの示す方向に進む
    // 突然今と180度逆の方向を示してくる場合もあるので注意
    // 止まらないと壁にぶつかる
    RobotMove(next_dir);  // RobotMove関数はDirection型を受け取ってロボットをそっちに動かす関数
  }
  RobotStop();
  HAL_Delay(100);
  return 0;
}

}  // namespace maze_run
