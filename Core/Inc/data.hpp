#ifndef CORE_INC_DATA_HPP_
#define CORE_INC_DATA_HPP_
#include <vector>
namespace data {
using geometry = struct geometry {
  float x = 0;
  float y = 0;
  float z = 0;
  geometry(float x, float y, float z) : x(x), y(y), z(z) {}
};
using twist = struct twist {
  float speed = 0;
  float angle_vel = 0;
  float sig = 0;
  twist(float speed, float angle_vel, float sig) : speed(speed), angle_vel(angle_vel), sig(sig) {}
  twist() : speed(0), angle_vel(0), sig(0) {}
};

using drive_record = struct drive_record {
  float speed = 0;
  float signal = 0;
  drive_record(float speed, float signal) : speed(speed), signal(signal) {}
  drive_record() : speed(0), signal(0) {}
};

using drive_records = std::vector<drive_record>;

}  // namespace data

#endif  // CORE_INC_DATA_HPP_