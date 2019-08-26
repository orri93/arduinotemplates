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
  virtual T value() = 0;
  virtual C code() = 0;
protected:
  ::gos::atl::type::range range_;
};

}
}

#endif