#ifndef CORE_INC_DATA_HPP_
#define CORE_INC_DATA_HPP_
#include <vector>
namespace data {
using geometry = struct geometry {
  float x;
  float y;
  float z;
  geometry(float x, float y, float z) : x(x), y(y), z(z) {}
};
using twist = struct twist {
  geometry linear;
  geometry angular;
  twist(geometry linear, geometry angular) : linear(linear), angular(angular) {}
};

using drive_records = struct drive_records {
  std::vector<twist> vel;
  drive_records(std::vector<twist> vel) : vel(vel) {}
};

}  // namespace data

#endif  // CORE_INC_DATA_HPP_