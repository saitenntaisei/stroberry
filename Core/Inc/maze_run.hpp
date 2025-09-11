#ifndef CORE_INC_MAZE_RUN_HPP_
#define CORE_INC_MAZE_RUN_HPP_

#include "../lib/MazeSolver2015/Agent.h"
#include "../lib/MazeSolver2015/Maze.h"

namespace maze_run {
// 探索した迷路の壁情報がはいる
extern Maze maze;
// 探索の指示を出す
extern Agent agent;
// 前回のAgentの状態を保存しとく
extern Agent::State prev_State;
extern IndexVec robot_position;
extern Direction robot_dir;
extern bool is_start_block;
extern int prev_wall_cnt;
extern Direction wall;

int search_run();
void robot_move(const Direction& dir);
void robot_stop();

const Direction& get_wall_data();
const IndexVec& get_robot_posion();

}  // namespace maze_run
#endif  // CORE_INC_MAZE_RUN_HPP_
