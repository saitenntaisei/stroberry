#ifndef CORE_INC_MAZE_RUN_HPP_
#define CORE_INC_MAZE_RUN_HPP_
#include "../lib/micromouse-maze-library/include/MazeLib/Maze.h"
#include "../lib/micromouse-maze-library/include/MazeLib/StepMap.h"
namespace maze_run {
using namespace MazeLib;

int SearchRun(Maze& maze, const Maze& mazeTarget);
int ShortestRun(const Maze& maze);
void MoveRobot(Direction dir);

int SearchRun(Maze& maze, const Maze& mazeTarget) {
  /* 探索テスト */
  StepMap stepMap;  //< 経路導出に使用するステップマップ
  /* 現在方向は、現在区画に向かう方向を表す。
   * 現在区画から出る方向ではないことに注意する。
   * +---+---+---+ 例
   * |   <       | <--- (0, 2, West)
   * +   +---+ ^ + <--- (2, 2, North)
   * |   >       | <--- (1, 1, East)
   * +   +---+ v + <--- (2, 0, South)
   * | S |       | <--- (0, 0)
   * +---+---+---+
   */
  Position currentPos = Position(0, 0);     //< 現在の区画位置
  Direction currentDir = Direction::North;  //< 現在向いている方向
  /* 1. ゴールへ向かう探索走行 */
  while (1) {
    /* 壁を確認。ここでは mazeTarget を参照しているが、実際には壁を見る */
    const bool wall_front = mazeTarget.isWall(currentPos, currentDir + Direction::Front);
    const bool wall_left = mazeTarget.isWall(currentPos, currentDir + Direction::Left);
    const bool wall_right = mazeTarget.isWall(currentPos, currentDir + Direction::Right);
    /* 迷路の壁を更新 */
    maze.updateWall(currentPos, currentDir + Direction::Front, wall_front);
    maze.updateWall(currentPos, currentDir + Direction::Left, wall_left);
    maze.updateWall(currentPos, currentDir + Direction::Right, wall_right);
    /* 現在地のゴール判定 */
    const auto& goals = maze.getGoals();
    if (std::find(goals.cbegin(), goals.cend(), currentPos) != goals.cend()) break;
    /* 現在地からゴールへの移動経路を、未知壁はないものとして導出 */
    const auto moveDirs = stepMap.calcShortestDirections(maze, currentPos, maze.getGoals(), false, true);
    /* エラー処理 */
    if (moveDirs.empty()) {
      MAZE_LOGE << "Failed to Find a path to goal!" << std::endl;
      return -1;
    }
    /* 未知壁のある区画に当たるまで進む */
    for (const auto nextDir : moveDirs) {
      /* 未知壁があったら終了 */
      if (maze.unknownCount(currentPos)) break;
      /* ロボットを動かす */
      const auto relativeDir = Direction(nextDir - currentDir);
      MoveRobot(relativeDir);
      /* 現在地を進める */
      currentPos = currentPos.next(nextDir);
      currentDir = nextDir;
    }
  }
  /* 2. 最短経路上の未知区画をつぶす探索走行 */
  while (1) {
    /* 壁を確認。ここでは mazeTarget を参照しているが、実際には壁を見る */
    const bool wall_front = mazeTarget.isWall(currentPos, currentDir + Direction::Front);
    const bool wall_left = mazeTarget.isWall(currentPos, currentDir + Direction::Left);
    const bool wall_right = mazeTarget.isWall(currentPos, currentDir + Direction::Right);
    /* 迷路の壁を更新 */
    maze.updateWall(currentPos, currentDir + Direction::Front, wall_front);
    maze.updateWall(currentPos, currentDir + Direction::Left, wall_left);
    maze.updateWall(currentPos, currentDir + Direction::Right, wall_right);
    /* 最短経路上の未知区画を洗い出し */
    const auto shortestDirs = stepMap.calcShortestDirections(maze, maze.getStart(), maze.getGoals(), false, false);
    Positions shortestCandidates;
    auto pos = maze.getStart();
    for (const auto nextDir : shortestDirs) {
      pos = pos.next(nextDir);
      if (maze.unknownCount(pos)) shortestCandidates.push_back(pos);
    }
    /* 最短経路上に未知区画がなければ次へ */
    if (shortestCandidates.empty()) break;
    /* 現在地から最短候補への移動経路を未知壁はないものとして導出 */
    const auto moveDirs = stepMap.calcShortestDirections(maze, currentPos, shortestCandidates, false, true);
    /* エラー処理 */
    if (moveDirs.empty()) {
      MAZE_LOGE << "Failed to Find a path to goal!" << std::endl;
      return -1;
    }
    /* 未知壁のある区画に当たるまで進む */
    for (const auto nextDir : moveDirs) {
      /* 未知壁があったら終了 */
      if (maze.unknownCount(currentPos)) break;
      /* ロボットを動かす */
      const auto relativeDir = Direction(nextDir - currentDir);
      MoveRobot(relativeDir);
      /* 現在地を進める */
      currentPos = currentPos.next(nextDir);
      currentDir = nextDir; /* アニメーション表示 */
    }
  }
  /* 3. スタート区画へ戻る走行 */
  while (1) {
    /* 現在地のスタート区画判定 */
    if (currentPos == maze.getStart()) break;
    /* 現在地からスタートへの最短経路を既知壁のみの経路で導出 */
    const auto moveDirs = stepMap.calcShortestDirections(maze, currentPos, {maze.getStart()}, true, true);
    /* エラー処理 */
    if (moveDirs.empty()) {
      MAZE_LOGE << "Failed to Find a path to goal!" << std::endl;
      return -1;
    }
    /* 経路上を進む */
    for (const auto nextDir : moveDirs) {
      /* ロボットを動かす */
      const auto relativeDir = Direction(nextDir - currentDir);
      MoveRobot(relativeDir);
      /* 現在地を進める */
      currentPos = currentPos.next(nextDir);
      currentDir = nextDir;
    }
  }
  /* 正常終了 */
  return 0;
}

/**
 * @brief 最短走行のアルゴリズム
 */
int ShortestRun(const Maze& maze) {
  /* スタートからゴールまでの最短経路導出 */
  StepMap stepMap;
  const auto shortestDirs = stepMap.calcShortestDirections(maze, maze.getStart(), maze.getGoals(), true, false);
  if (shortestDirs.empty()) {
    MAZE_LOGE << "Failed to Find a path to goal!" << std::endl;
    return -1;
  }
  /* 最短走行 */
  Position currentPos = maze.getStart();
  Direction currentDir = Direction::North;
  for (const auto nextDir : shortestDirs) {
    /* ロボットを動かす */
    const auto relativeDir = Direction(nextDir - currentDir);
    MoveRobot(relativeDir);
    /* 現在地を進める */
    currentPos = currentPos.next(nextDir);
    currentDir = nextDir;
  }
  /* 最短経路の表示 */
  maze.print(shortestDirs);
  /* 終了 */
  return 0;
}
void MoveRobot(Direction dir) { return; }

}  // namespace maze_run
#endif