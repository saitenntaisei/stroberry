#include "../Inc/maze_run.hpp"

#include <cstdio>
#include <cstring>

#include "../../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"
#include "../Inc/flash.hpp"
#include "../lib/MazeSolver2015/MazeSolver_conf.h"

namespace maze_run {
// 変数の定義
Maze maze;
Agent agent(maze);
Agent::State prev_state = Agent::State::IDLE;
IndexVec robot_position(0, 0);
Direction robot_dir(NORTH);
bool is_start_block = true;
int prev_wall_cnt = 0;
Direction wall;

// 関数の実装
const IndexVec& GetRobotPosition() {
  // 絶対座標系で返す
  return robot_position;
}

int SearchRun() {
  auto save_maze_to_flash = []() -> int {
    char asciiData[MAZE_SIZE + 1][MAZE_SIZE + 1];
    maze.saveToArray(asciiData);
    std::memcpy(flash::work_ram, asciiData, sizeof(asciiData));
    if (!flash::Store()) {
      printf("flash store error\r\n");
      return -1;
    }
    return 0;
  };

  while (1) {
    // センサから取得した壁情報を入れる
    const Direction wallData = GetWallData();
    // ロボットの座標を取得
    const IndexVec robotPos = GetRobotPosition();

    // 壁情報を更新 次に進むべき方向を計算
    agent.update(robotPos, wallData);
    // Agentの状態を確認
    // FINISHEDになったら計測走行にうつる
    if (agent.getState() == Agent::FINISHED) {
      if (save_maze_to_flash() != 0) {
        return -1;
      }
      break;
    }

    // ゴールにたどり着いた瞬間に一度だけmazeのバックアップをとる
    // Mazeクラスはoperator=が定義してあるからa = bでコピーできる
    if (prev_state == Agent::SEARCHING_NOT_GOAL && agent.getState() != Agent::SEARCHING_NOT_GOAL) {
      if (save_maze_to_flash() != 0) {
        return -1;
      }
    }
    prev_state = agent.getState();

    // Agentの状態が探索中の場合は次に進むべき方向を取得する
    const Direction& nextDir = agent.getNextDirection();

    // nextDirの示す方向に進む
    // 突然今と180度逆の方向を示してくる場合もあるので注意
    // 止まらないと壁にぶつかる
    RobotMove(nextDir);  // RobotMove関数はDirection型を受け取ってロボットをそっちに動かす関数
  }
  maze_run::RobotStop();
  HAL_Delay(100);
  return 0;
}

}  // namespace maze_run
