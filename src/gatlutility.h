#ifndef _GOS_ARDUINO_TEMPLATE_LIBRARY_UTILITY_H_
#define _GOS_ARDUINO_TEMPLATE_LIBRARY_UTILITY_H_

#include <Arduino.h>

#include <gatltype.h>

namespace gos {
namespace atl {
namespace utility {

template<typename T> T restrict(const T& value, const T& minimum, const T& maximum) {
  if (value >= minimum && value <= maximum) {
    return value;
  } else if (value > maximum) {
    return maximum;
  } else {
    return minimum;
  }
}

template<typename T> T restrict(const T& value, const ::gos::atl::type::range<T>& range) {
  return restrict(value, range.lowest, range.highest);
}

}
}
}

#endif
