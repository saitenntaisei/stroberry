#ifndef CORE_INC_PARTS_HPP_
#define CORE_INC_PARTS_HPP_

namespace parts {
template <class LEFT, class RIGHT>
struct wheel {
  LEFT left;
  RIGHT right;
};

enum class Run_mode_t : uint8_t { STRAIGHT_MODE, TURN_MODE };

}  // namespace parts
#endif  // CORE_INC_PARTS_HPP_
