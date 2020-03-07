#ifndef _GOS_ARDUINO_TEMPLATE_LIBRARY_DHT_H_
#define _GOS_ARDUINO_TEMPLATE_LIBRARY_DHT_H_

#include <Arduino.h>

#define GATL_DHT_SUCCESS            0x00
#define GATL_DHT_TIMEOUT            0x01
#define GATL_DHT_CHECKSUM           0x02

/* data sheet says "at least 1 ms" */
#define GATL_DHT_DELAY_GENERAL      1100
#define GATL_DHT_DELAY_DHT11       20000

#define GATL_DHT_INTERVAL_GENERAL   2000
#define GATL_DHT_INTERVAL_DHT11     1000

#define        GATL_DHT_DEFAULT_PIN_TYPE uint8_t
#define        GATL_DHT_DEFAULT_RAW_TYPE uint16_t
#define GATL_DHT_DEFAULT_VALUE_REAL_TYPE float
#define  GATL_DHT_DEFAULT_VALUE_INT_TYPE int32_t
#define      GATL_DHT_DEFAULT_DELAY_TYPE uint16_t
#define    GATL_DHT_DEFAULT_RESAULT_TYPE uint8_t
#define      GATL_DHT_DEFAULT_TIMER_TYPE unsigned long

namespace gos {
namespace atl {
namespace dht {

template <typename V> struct Values {
  V temperature;
  V humidity;
};

template <typename R = GATL_DHT_DEFAULT_RAW_TYPE>
  GATL_DHT_DEFAULT_RESAULT_TYPE read(
    struct Values<R>& values,
    const GATL_DHT_DEFAULT_PIN_TYPE& pin,
    const GATL_DHT_DEFAULT_DELAY_TYPE& dms = GATL_DHT_DELAY_GENERAL) {
  GATL_DHT_DEFAULT_TIMER_TYPE tick;
  uint8_t age;
  int8_t i;
  R raw;

  // Send start signal.  See DHT datasheet for full signal diagram:
  //   http://www.adafruit.com/datasheets/Digital%20humidity%20and%20temperature%20sensor%20AM2302.pdf

  // Go into high impedence state to let pull-up raise data line level and
  // start the reading process.
  pinMode(pin, INPUT_PULLUP);
  delay(1);

  pinMode(pin, OUTPUT);
  delayMicroseconds(dms);

  pinMode(pin, INPUT_PULLUP); // Switch bus to receive data

  // We're going to read 83 edges:
  // - First a FALLING, RISING, and FALLING edge for the start bit
  // - Then 40 bits: RISING and then a FALLING edge per bit
  // To keep our code simple, we accept any HIGH or LOW reading if it's max 85 usecs long
  values.temperature = 0;
  values.humidity = 0;
  raw = 0;

  pinMode(pin, OUTPUT);

  // Turn off interrupts temporarily because the next sections
  // are timing critical and we don't want any interruptions.
  noInterrupts();

  for (i = -3; i < 2 * 40; i++) {
    tick = micros();
    do {
      age = (unsigned long)(micros() - tick);
      if (age > 90) {
        return GATL_DHT_TIMEOUT;
      }
    } while (digitalRead(pin) == (i & 1));

    if (i >= 0 && (i & 1)) {
      // Now we are being fed our 40 bits
      raw <<= 1;

      // A zero max 30 usecs, a one at least 68 usecs.
      if (age > 30) {
        raw |= 1; // we got a one
      }
    }

    switch (i) {
    case 31:
      values.humidity = raw;
      break;
    case 63:
      values.temperature = raw;
      raw = 0;
      break;
    }
  }

  interrupts();

  // Verify checksum
  if (
    (uint8_t)(((uint8_t)(values.humidity)) +
    ((values.humidity) >> 8) +
      ((uint8_t)(values.temperature)) +
      ((values.temperature) >> 8)) != raw) {
    return GATL_DHT_CHECKSUM;
  }

  return GATL_DHT_SUCCESS;
}

namespace convert {
namespace real {
template <
  typename V = GATL_DHT_DEFAULT_VALUE_REAL_TYPE,
  typename R = GATL_DHT_DEFAULT_RAW_TYPE>
  void general(struct Values<V>& values, const struct Values<R>& raw) {
  values.humidity = raw.humidity * 0.1;
  values.temperature = raw.temperature & 0x8000 ?
    ((V)(-(int16_t)(raw.temperature & 0x7FFF)) * 0.1) :
    ((V)((int16_t)(raw.temperature)) * 0.1);
}
template <
  typename V = GATL_DHT_DEFAULT_VALUE_REAL_TYPE,
  typename R = GATL_DHT_DEFAULT_RAW_TYPE>
  void dht11(struct Values<V>& values, const struct Values<R>& raw) {
  values.humidity = raw.humidity >> 8;
  values.temperature = raw.temperature >> 8;
}
} // namespace real
namespace integer {
template <
  typename V = GATL_DHT_DEFAULT_VALUE_INT_TYPE,
  typename R = GATL_DHT_DEFAULT_RAW_TYPE>
  void general(struct Values<V>& values, const struct Values<R>& raw) {
  values.humidity = raw.humidity * 0.1;
  values.temperature = raw.temperature & 0x8000 ?
    (V)(-(int16_t)(raw.temperature & 0x7FFF)) :
    (V)((int16_t)(raw.temperature));
}
template <
  typename V = GATL_DHT_DEFAULT_VALUE_INT_TYPE,
  typename R = GATL_DHT_DEFAULT_RAW_TYPE>
  void dht11(struct Values<V>& values, const struct Values<R>& raw) {
  values.humidity = raw.humidity >> 8;
  values.temperature = raw.temperature >> 8;
}
} // namespace real
} // namespace convert

} // namespace dht
} // namespace atl
} // namespace gos

#endif
