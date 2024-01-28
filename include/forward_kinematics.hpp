#pragma once

#include "array"

namespace fast_fk {
  template<std::size_t SIZE>
  void forward_kinematics(std::array<double, SIZE> &input_data);
}
