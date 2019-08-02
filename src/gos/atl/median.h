#ifndef _GOS_ARDUINO_TEMPLATE_LIBRARY_MEDIAN_H_
#define _GOS_ARDUINO_TEMPLATE_LIBRARY_MEDIAN_H_

#include <Arduino.h>

#include <gos/atl/sort.h>

// should at least be 5 to be practical
// odd size results in a 'real' middle element.
// even size takes the lower of the two middle elements
#ifndef MEDIAN_DEFAULT_COUNT
#define MEDIAN_DEFAULT_COUNT 8
#endif

namespace gos {
namespace atl {

template<
  typename T,
  typename R = uint8_t,
  typename C = uint8_t,
  typename I = uint8_t >
class Median {
public:
  Median(const T& undefined, const C& size = MEDIAN_DEFAULT_COUNT) :
    size_(size),
    count_(),
    index_(size),
    values_(nullptr),
    reference_(nullptr),
    undefined_(undefined),
    median_(),
    cached_(nullptr)
  {
    values_ = static_cast<T*>(malloc(count_ * sizeof(T)));
    reference_ = static_cast<R*>(malloc(count_ * sizeof(R)));
    resetref();
  }

  void clear() {
    count_ = 0;
    cached_ = nullptr;
    resetref();
  }

  bool add(const T& value) {
    cached_ = nullptr;
    index_ = index_ < size_ - 1 ? index_ + 1 : 0;
    values_[index_] = value;
    if (count_ < size_) {
      count_++;
      return false;
    } else {
      return true;
    }
  }

  T get() {
    if (cached_) {
      return *cached_;
    } else {
      cached_ = &median_;
      switch (count_) {
      case 0:
        return median_ = undefined_;
      case 1:
        return median_ = values_[0];
      case 2:
        return median_ = (values_[0] + values_[1]) / static_cast<T>(2);
      default:
        ::gos::atl::sort::insertion<T, R>(values_, reference_, count_);
        if (count_ & 1)
          return median_ = values_[reference_[count_ / 2]];
        else
          return median_ =
          (values_[reference_[count_ / 2]] + values_[reference_[count_ / 2 - 1]]) / 2.0F;
      }
    }
  }

  void cleanup() {
    if (values_) {
      free(values_);
      values_ = nullptr;
    }
    if (reference_) {
      free(reference_);
      reference_ = nullptr;
    }
  }

private:
  void resetref() {
    for (index_ = 0; index_ < size_; index_++) {
      reference_[index_] = static_cast<R>(index_);
    }
  }
  C size_;
  C count_;
  I index_;
  T* values_;
  R* reference_;
  T undefined_;
  T median_;
  T* cached_;
};

}
}
#endif /* _GOS_ARDUINO_TEMPLATE_LIBRARY_SORT_H_ */
