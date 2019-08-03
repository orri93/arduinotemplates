#ifndef _GOS_ARDUINO_TEMPLATE_LIBRARY_MEDIAN_H_
#define _GOS_ARDUINO_TEMPLATE_LIBRARY_MEDIAN_H_

#include <Arduino.h>

#include <gos/atl/statistics.h>
#include <gos/atl/sort.h>

namespace gos {
namespace atl {
namespace statistics {

template<typename T, typename I = uint8_t >
class Median {
public:
  Median(::gos::atl::statistics::Set<T, I>& set) :
    set_(set),
    reference_(nullptr) {
    reference_ = static_cast<I*>(malloc(set_.Size * sizeof(I)));
    resetref();
  }

  void clear() {
    set_.clear();
    resetref();
  }

  T get() {
    if (set_.IsCached) {
      return set_.Statistics;
    }
    else {
      set_.IsCached = true;
      if (set_.Count > 2) {
        ::gos::atl::sort::insertion<T, I>(set_.Values, reference_, set_.Count);
        if (set_.Count & 1) {
          return set_.Statistics = set_.Values[reference_[set_.Count / 2]];
        }
        else {
          return set_.Statistics = (
            set_.Values[reference_[set_.Count / 2]] +
            set_.Values[reference_[set_.Count / 2 - 1]])
            / static_cast<T>(2);
        }
      }
      else {
        return set_.Statistics = set_.get();
      }
    }
  }

  void cleanup() {
    set_.cleanup();
    if (reference_) {
      free(reference_);
      reference_ = nullptr;
    }
  }

private:
  void resetref() {
    for (set_.Index = 0; set_.Index < set_.Size; set_.Index++) {
      reference_[set_.Index] = static_cast<I>(set_.Index);
    }
  }
  ::gos::atl::statistics::Set<T, I>& set_;
  I* reference_;
};

}
}
}
#endif /* _GOS_ARDUINO_TEMPLATE_LIBRARY_MEDIAN_H_ */
