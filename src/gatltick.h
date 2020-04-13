#ifndef _GOS_ARDUINO_TEMPLATE_LIBRARY_TICK_H_
#define _GOS_ARDUINO_TEMPLATE_LIBRARY_TICK_H_

#include <Arduino.h>

#define GATL_TICK_DEFAULT_TYPE unsigned long
#define GATL_TICK_DEFAULT_INTERVAL 1000


namespace gos {
namespace atl {

template<typename T = GATL_TICK_DEFAULT_TYPE, typename I = T> class Tick {
public:
  Tick() : Interval(GATL_TICK_DEFAULT_INTERVAL), Next(0) {
  }
  Tick(const I& interval) : Interval(interval), Next(0) {
  }
  bool is(const T& tick) {
    if (tick < Next) {
      return false;
    } else {
      Next = tick + Interval;
      return true;
    }
  }
  I Interval;
  T Next;
};

namespace tick {
namespace is {
template<typename T = GATL_TICK_DEFAULT_TYPE, typename I = T>
bool next(T& next, const T& tick, const I& interval) {
  if (tick < next) {
    return false;
  }
  else {
    next = tick + interval;
    return true;
  }
}
} // namespace is
} // namespace tick
} // namespace atl
} // namespace gos

#endif
