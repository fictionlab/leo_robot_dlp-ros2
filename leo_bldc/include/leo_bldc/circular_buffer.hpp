#pragma once

#include <cstddef>

namespace leo_bldc
{

template<class T, size_t SIZE>
class CircularBuffer {
  T values_[SIZE];
  size_t iter_ = 0;

public:
  void push_back(T val)
  {
    values_[iter_++] = val;
    if (iter_ >= SIZE) {
      iter_ = 0;
      filled_ = true;
    }
  }

  T get_recent() const
  {
    return values_[(iter_ - 1) % SIZE];
  }

  T get_oldest() const
  {
    return values_[iter_];
  }

  bool filled_{};
};

}  // namespace leo_bldc
