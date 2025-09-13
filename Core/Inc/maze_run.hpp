#ifndef CORE_INC_MAZE_RUN_HPP_
#define CORE_INC_MAZE_RUN_HPP_

#include "../lib/MazeSolver2015/Agent.h"
#include "../lib/MazeSolver2015/Maze.h"

namespace maze_run {
class MazeRunner {
 public:
  MazeRunner() : agent_(maze_), robot_position_(0, 0), robot_dir_(NORTH), wall_(0) {}

  int SearchRun();
  int TrueRun();

  void RobotMove(const Direction& dir);
  void RobotMove(const Operation& op);
  void RobotStop();
  void LoadMaze4TrueRun(Maze& maze);

  const Direction& GetWallData();
  const IndexVec& GetRobotPosition() const { return robot_position_; }

 private:
  Maze maze_;
  Agent agent_;
  Agent::State prev_state_ = Agent::State::IDLE;
  IndexVec robot_position_;
  Direction robot_dir_;
  bool is_start_block_ = true;
  int prev_wall_cnt_ = 0;
  Direction wall_;
};

}  // namespace maze_run
#endif  // CORE_INC_MAZE_RUN_HPP_
