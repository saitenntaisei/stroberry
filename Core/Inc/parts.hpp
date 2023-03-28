#ifndef MY_PART_HPP
#define MY_PART_HPP

namespace parts {
template <class LEFT, class RIGHT>
struct wheel {
  LEFT left;
  RIGHT right;
};

}  // namespace parts
#endif