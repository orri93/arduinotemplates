#ifndef _GOS_ARDUINO_TEMPLATE_LIBRARY_TYPE_H_
#define _GOS_ARDUINO_TEMPLATE_LIBRARY_TYPE_H_

#include <Arduino.h>

namespace gos {
namespace atl {
namespace type {

template<typename T> struct range {
  T lowest;
  T highest;
};

template<typename T> range<T> make_range(const T& lowest, const T& highest) {
  range<T> range;
  range.lowest = lowest;
  range.highest = highest;
  return range;
}

}
}
}

#endif
