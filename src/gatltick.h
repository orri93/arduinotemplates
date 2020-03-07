#ifndef _GOS_ARDUINO_TEMPLATE_LIBRARY_TICK_H_
#define _GOS_ARDUINO_TEMPLATE_LIBRARY_TICK_H_

#include <Arduino.h>

#define GATL_TICK_DEFAULT_TYPE unsigned long

namespace gos {
namespace atl {
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
