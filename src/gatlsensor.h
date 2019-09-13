#ifndef _GOS_ARDUINO_TEMPLATE_LIBRARY_SENSOR_H_
#define _GOS_ARDUINO_TEMPLATE_LIBRARY_SENSOR_H_

#include <Arduino.h>

#include <gatltype.h>

namespace gos {
namespace atl {

template<typename T = double, typename C = uint8_t> class Sensor {
public:
  enum class Status {
    Undefined,
    Operational,
    AboveRange,
    BelowRange,
    Fault
  };
  virtual ~Sensor() {}
  virtual void begin() {}
  virtual Status measure() = 0;
  virtual const char* error(uint8_t& length) = 0;
  C Code;
  T Value;
  ::gos::atl::type::range<T> Range;
  Status Last;
protected:
  virtual Status check(const bool& restrictvalue = true) {
    if (Value >= Range.lowest && Value <= Range.highest) {
      return Last = Status::Operational;
    } else if (Value < Range.lowest) {
      if (restrictvalue)
        Value = Range.lowest;
      return Last = Status::BelowRange;
    } else {
      if (restrictvalue)
        Value = Range.highest;
      return Last = Status::AboveRange;
    }
  }
};

}
}

#endif