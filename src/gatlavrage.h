#ifndef _GOS_ARDUINO_TEMPLATE_LIBRARY_AVRAGE_H_
#define _GOS_ARDUINO_TEMPLATE_LIBRARY_AVRAGE_H_

#include <Arduino.h>

#include <gatlstatistics.h>

namespace gos {
namespace atl {
namespace statistics {

template<typename T, typename I = uint8_t>
class Avrage {
public:
  Avrage(::gos::atl::statistics::Set<T, I>& set) :
    set_(set),
    sum_() {
  }

  void clear() {
    set_.clear();
    sum_ = T();
  }

  void add(const T& value) {
    if (set_.Count >= set_.Size) {
      sum_ -= set_.Values[set_.Index < set_.Size - 1 ? set_.Index + 1 : 0];
    }
    sum_ += value;
    set_.add(value);
  }

  T get() {
    if (set_.IsCached) {
      return set_.Statistics;
    }
    else {
      set_.IsCached = true;
      if (set_.Count > 2) {
        return set_.Statistics = sum_ / static_cast<T>(set_.Count);
      }
      else {
        return set_.Statistics = set_.get();
      };
    }
  }

private:
  ::gos::atl::statistics::Set<T, I>& set_;
  T sum_;
};

}
}
}
#endif /* _GOS_ARDUINO_TEMPLATE_LIBRARY_AVRAGE_H_ */
