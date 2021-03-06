#ifndef _GOS_ARDUINO_TEMPLATE_LIBRARY_LED_H_
#define _GOS_ARDUINO_TEMPLATE_LIBRARY_LED_H_

#include <Arduino.h>

#define GOS_ARDUINO_TYPE_PIN uint8_t
#define GOS_ARDUINO_TYPE_DEFAULT_LED uint8_t
#define GOS_ARDUINO_TYPE_DEFAULT_SIN float
#define GOS_ARDUINO_TYPE_TIME float


#ifndef GOS_ARDUINO_LED_SIN_HEARTBEAT_STEPS
#define GOS_ARDUINO_LED_SIN_HEARTBEAT_STEPS 0.001F
#endif

#ifndef GOS_ARDUINO_LED_SIN_START
#define GOS_ARDUINO_LED_SIN_START -HALF_PI
#endif

#ifndef GOS_ARDUINO_LED_SIN_MAXIMUM_AT
#define GOS_ARDUINO_LED_SIN_MAXIMUM_AT TWO_PI
#endif

#ifndef GOS_ARDUINO_LED_SIN_MAXIMUM_HALF
#define GOS_ARDUINO_LED_SIN_MAXIMUM_HALF 0x7f
#endif

#ifndef GOS_ARDUINO_LED_SIN_MAXIMUM_LOOP
#define GOS_ARDUINO_LED_SIN_MAXIMUM_LOOP PI + HALF_PI
#endif


namespace gos {
namespace atl {
namespace led {

inline void initialize(const GOS_ARDUINO_TYPE_PIN& pin) {
  pinMode(pin, OUTPUT);
}

template<typename T = GOS_ARDUINO_TYPE_DEFAULT_LED>
void blink(
  const GOS_ARDUINO_TYPE_PIN& pin,
  T count = 3,
  const unsigned long& delayms = 250) {
  while (count > 0) {
    digitalWrite(pin, HIGH);
    delay(delayms);
    digitalWrite(pin, LOW);
    delay(delayms);
    count--;
  }
}

namespace sin {

template<typename T = GOS_ARDUINO_TYPE_DEFAULT_SIN> void initialize(T& at) {
  at = GOS_ARDUINO_LED_SIN_START;
}

template<typename T = GOS_ARDUINO_TYPE_DEFAULT_SIN>
GOS_ARDUINO_TYPE_DEFAULT_LED output(const T& at) {
  return static_cast<GOS_ARDUINO_TYPE_DEFAULT_LED>(
    ::sin(at) * GOS_ARDUINO_LED_SIN_MAXIMUM_HALF +
    GOS_ARDUINO_LED_SIN_MAXIMUM_HALF);
}

template<typename T = GOS_ARDUINO_TYPE_DEFAULT_SIN>
GOS_ARDUINO_TYPE_DEFAULT_LED output(const T& at, T max) {
  max /= T(2);
  return static_cast<GOS_ARDUINO_TYPE_DEFAULT_LED>(::sin(at) * max + max);
}

template<typename T = GOS_ARDUINO_TYPE_DEFAULT_SIN>
void step(T& at, const T& step = GOS_ARDUINO_LED_SIN_HEARTBEAT_STEPS) {
  at = at < GOS_ARDUINO_LED_SIN_MAXIMUM_AT ? at + step : T();
}

template<typename T = GOS_ARDUINO_TYPE_DEFAULT_SIN>
void loop(
  const GOS_ARDUINO_TYPE_PIN& pin,
  T& at,
  const T& step = GOS_ARDUINO_LED_SIN_HEARTBEAT_STEPS) {
  analogWrite(pin, output<T>(at));
  ::gos::atl::led::sin::step<T>(at, step);
}

namespace full {

template<typename T = GOS_ARDUINO_TYPE_DEFAULT_SIN> void cycle(
  const GOS_ARDUINO_TYPE_PIN& pin,
  T at = GOS_ARDUINO_LED_SIN_START,
  const T& step = GOS_ARDUINO_LED_SIN_HEARTBEAT_STEPS) {
  while (at < GOS_ARDUINO_LED_SIN_MAXIMUM_LOOP) {
    analogWrite(pin, ::gos::atl::led::sin::output<T>(at));
    at += step;
  }
  digitalWrite(pin, LOW);
}

template<typename T = GOS_ARDUINO_TYPE_DEFAULT_SIN> void cycle(
  const GOS_ARDUINO_TYPE_PIN& pin,
  uint8_t count,
  T at = GOS_ARDUINO_LED_SIN_START,
  const T& step = GOS_ARDUINO_LED_SIN_HEARTBEAT_STEPS) {
  while (count > 0) {
    while (at < GOS_ARDUINO_LED_SIN_MAXIMUM_LOOP) {
      analogWrite(pin, ::gos::atl::led::sin::output<T>(at));
      at += step;
    }
    at = GOS_ARDUINO_LED_SIN_START;
    --count;
  }
  digitalWrite(pin, LOW);
}

template<typename T = GOS_ARDUINO_TYPE_DEFAULT_SIN> void cycle(
  const GOS_ARDUINO_TYPE_PIN& pin,
  const unsigned long& delayms,
  uint8_t count,
  T at = GOS_ARDUINO_LED_SIN_START,
  const T& step = GOS_ARDUINO_LED_SIN_HEARTBEAT_STEPS) {
  while(count > 0) {
    while (at < GOS_ARDUINO_LED_SIN_MAXIMUM_LOOP) {
      analogWrite(pin, ::gos::atl::led::sin::output<T>(at));
      delay(delayms);
      at += step;
    }
    at = GOS_ARDUINO_LED_SIN_START;
    --count;
  }
  digitalWrite(pin, LOW);
}

}

}

}
}
}

#endif
