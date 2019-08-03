#ifndef _GOS_ARDUINO_TEMPLATE_LIBRARY_STATISTICS_H_
#define _GOS_ARDUINO_TEMPLATE_LIBRARY_STATISTICS_H_

#include <Arduino.h>

// should at least be 5 to be practical
// odd size results in a 'real' middle element.
// even size takes the lower of the two middle elements
#ifndef GOS_ATL_DEFAULT_STATISTICS_SET_COUNT
#define GOS_ATL_DEFAULT_STATISTICS_SET_COUNT 8
#endif

namespace gos {
namespace atl {
namespace statistics {

template<typename T, typename I = uint8_t>
class Set {
public:
  Set(const T& undefined = T(), const I& size = GOS_ATL_DEFAULT_STATISTICS_SET_COUNT) :
    Size(size),
    Count(),
    Index(size),
    undefined_(undefined),
    Values(nullptr),
    IsCached(false) {
    Values = static_cast<T*>(malloc(Size * sizeof(T)));
  }

  void clear() {
    Count = I();
    Index = Size;
    IsCached = false;
  }

  void add(const T& value) {
    IsCached = false;
    Index = Index < Size - 1 ? Index + 1 : 0;
    Values[Index] = value;
    if (Count < Size) {
      Count++;
    }
  }

  T get() {
    switch (Count) {
    case 1:
      return Values[0];
    case 2:
      return (Values[0] + Values[1]) / static_cast<T>(2);
    default:
      return undefined_;
    }
  }

  void cleanup() {
    if (Values) {
      free(Values);
      Values = nullptr;
    }
  }

  I Size;
  I Count;
  I Index;
  T Statistics;
  T* Values;
  bool IsCached;
protected:
  T undefined_;
};

}
}
}
#endif /* _GOS_ARDUINO_TEMPLATE_LIBRARY_STATISTICS_H_ */
