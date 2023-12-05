#ifndef CORE_INC_PARTS_HPP_
#define CORE_INC_PARTS_HPP_
#include <cstdint>
namespace parts {
template <class LEFT, class RIGHT>
struct wheel {
  LEFT left = NULL;
  RIGHT right = NULL;
};

enum class RunModeT : std::uint8_t { STRAIGHT_MODE, TURN_MODE, STOP_MODE };

}  // namespace parts
#endif  // CORE_INC_PARTS_HPP_
