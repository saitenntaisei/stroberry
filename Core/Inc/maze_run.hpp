#ifndef CORE_INC_MAZE_RUN_HPP_
#define CORE_INC_MAZE_RUN_HPP_
#include <functional>

#include "../lib/MazeSolver2015/Agent.h"
#include "../lib/MazeSolver2015/Maze.h"
#include "../lib/MazeSolver2015/mazeData.h"

namespace maze_run {
// 探索した迷路の壁情報がはいる
Maze maze;
// クラッシュした時のためのバックアップ
Maze maze_backup;
// 探索の指示を出す
Agent agent(maze);
// 前回のAgentの状態を保存しとく
Agent::State prev_State = Agent::State::IDLE;
IndexVec robot_position(0, 0);
Direction robot_dir(NORTH);
bool is_start_block = true;
uint8_t prev_wall_cnt = 0;
bool conditional_side_wall_control = false;

int search_run();
void robot_move(Direction dir);
void TurnRobot(Direction dir);

Direction get_wall_data();
IndexVec get_robot_posion();

IndexVec get_robot_posion() {
  // 絶対座標系で返す
  return robot_position;
}

int search_run() {
  while (1) {
    // センサから取得した壁情報を入れる
    Direction wallData = get_wall_data();
    // ロボットの座標を取得
    IndexVec robotPos = get_robot_posion();

    // 壁情報を更新 次に進むべき方向を計算
    agent.update(robotPos, wallData);

    // Agentの状態を確認
    // FINISHEDになったら計測走行にうつる
    if (agent.getState() == Agent::FINISHED) break;

    // ゴールにたどり着いた瞬間に一度だけmazeのバックアップをとる
    // Mazeクラスはoperator=が定義してあるからa = bでコピーできる
    if (prev_State == Agent::SEARCHING_NOT_GOAL && agent.getState() == Agent::SEARCHING_REACHED_GOAL) {
      maze_backup = maze;
    }
    prev_State = agent.getState();

    // Agentの状態が探索中の場合は次に進むべき方向を取得する
    Direction nextDir = agent.getNextDirection();

    // nextDirの示す方向に進む
    // 突然今と180度逆の方向を示してくる場合もあるので注意
    // 止まらないと壁にぶつかる
    robot_move(nextDir);  // robotMove関数はDirection型を受け取ってロボットをそっちに動かす関数
  }
  return 0;
}

}  // namespace maze_run
#endif