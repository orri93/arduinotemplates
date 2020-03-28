#ifndef _GOS_ARDUINO_TEMPLATE_LIBRARY_SENSOR_H_
#define _GOS_ARDUINO_TEMPLATE_LIBRARY_SENSOR_H_

#include <Arduino.h>

#include <gatltype.h>

namespace gos {
namespace atl {

namespace sensor {
enum class Status {
  Undefined,
  Operational,
  AboveRange,
  BelowRange,
  Fault
};
}

template<typename T = double, typename C = uint8_t> class Sensor {
public:
  virtual ~Sensor() {}
  virtual void begin() {}
  virtual ::gos::atl::sensor::Status measure() = 0;
  virtual const char* error(uint8_t& length) = 0;
  C Code;
  T Value;
  ::gos::atl::type::range<T> Range;
  ::gos::atl::sensor::Status Last;
protected:
  virtual ::gos::atl::sensor::Status check(const bool& restrictvalue = true) {
    if (Value >= Range.lowest && Value <= Range.highest) {
      return Last = ::gos::atl::sensor::Status::Operational;
    } else if (Value < Range.lowest) {
      if (restrictvalue)
        Value = Range.lowest;
      return Last = ::gos::atl::sensor::Status::BelowRange;
    } else {
      if (restrictvalue)
        Value = Range.highest;
      return Last = ::gos::atl::sensor::Status::AboveRange;
    }
  }
};

}
}

#endif