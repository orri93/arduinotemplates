#ifndef _GOS_ARDUINO_TEMPLATE_LIBRARY_UTILITY_H_
#define _GOS_ARDUINO_TEMPLATE_LIBRARY_UTILITY_H_

#include <Arduino.h>

#include <gatltype.h>
#include <gatlbuffer.h>

#ifdef GOS_ARDUINO_TEMPLATE_LIBRARY_UTILITY_VECTOR_SUPPORT
#include <vector>
#endif

namespace gos {
namespace atl {
namespace utility {

namespace crc {
template<typename C, typename S>
C read(::gos::atl::buffer::Holder<S, char>& buffer, const S& length) {
  return R();
}
template<typename C = uint16_t, typename I = uint16_t, typename S = uint16_t>
C calculate(::gos::atl::buffer::Holder<S, char>& buffer, const S& length) {
  I i, j;
  C crc = 0xFFFF;
  C tmp;

  // calculate crc
  for (i = 0; i < length; i++)
  {
    crc = crc ^ buffer.Buffer[i];

    for (j = 0; j < 8; j++)
    {
      tmp = crc & 0x0001;
      crc = crc >> 1;
      if (tmp)
      {
        crc = crc ^ 0xA001;
      }
    }
  }

  return crc;
}
}

namespace changed {
namespace apply {
template<typename T>
bool is(
  const T& value,
  ::gos::atl::type::optional<T>& optional,
  const T& threshold) {
  if (optional.is()) {
    if (abs(value - optional.get()) > threshold) {
      optional.assign(value);
      return true;
    } else {
      return false;
    }
  } else {
    optional.assign(value);
    return true;
  }
}
template<typename T>
bool is(const T& value, ::gos::atl::type::optional<T>& optional) {
  if (optional.isunequal(value)) {
    optional.assign(value);
    return true;
  }
  return false;
}
}
template<typename T>
bool is(
  const T& value,
  const ::gos::atl::type::optional<T>& optional,
  const T& threshold) {
  if (optional.is()) {
    if (abs(value - optional.get()) > threshold) {
      return true;
    }
  }
  return false;
}
template<typename T>
bool is(const T& value, const ::gos::atl::type::optional<T>& optional) {
  return optional.isunequal(value);
}
}

namespace range {

template<typename T> bool isinside(
  const T& value,
  const T& lowest,
  const T& highest) {
  return value >= lowest && value <= highest;
}

template<typename T> bool isinside(
  const T& value,
  ::gos::atl::type::range<T>& range) {
  return isinside(value, range.lowest, range.highest);
}

#ifdef GOS_ARDUINO_TEMPLATE_LIBRARY_UTILITY_VECTOR_SUPPORT
template<typename T> bool isoneinside(
  std::vector<T>& values,
  const T& lowest,
  const T& highest) {
  for (std::vector<T>::iterator it = values.begin(); it != values.end(); it++) {
    if (isinside(*it, lowest, highest)) {
      return true;
    }
  }
  return false;
}

template<typename T> bool isoneinside(
  std::vector<T>& values,
  ::gos::atl::type::range<T>& range) {
  for (std::vector<T>::iterator it = values.begin(); it != values.end(); it++) {
    if (isinside(*it, range)) {
      return true;
    }
  }
  return false;
}
#endif

template<typename T> bool ismemberof(
  const T& value,
  const T& first,
  const T& length) {
  return value >= first && value < first + length;
}

template<typename T> bool ismemberof(
  const T& value,
  ::gos::atl::type::range<T>& range) {
  return ismemberof(value, range.lowest, range.highest);
}

template<typename T, typename I> bool isonememberof(
  const T* values,
  const I& count,
  const T& first,
  const T& length) {
  for (I i = 0; i < count; i++) {
    if (ismemberof(values[i], first, length)) {
      return true;
    }
  }
  return false;
}

template<typename T, typename I> bool isonememberof(
  const T* values,
  const I& count,
  ::gos::atl::type::range<T>& range) {
  for (I i = 0; i < count; i++) {
    if (ismemberof(values[i], range)) {
      return true;
    }
  }
  return false;
}

#ifdef GOS_ARDUINO_TEMPLATE_LIBRARY_UTILITY_VECTOR_SUPPORT
template<typename T> bool isonememberof(
  const std::vector<T>& values,
  const T& first,
  const T& length) {
  for (std::vector<T>::iterator it = values.begin(); it != values.end(); it++) {
    if (ismemberof(*it, first, length)) {
      return true;
    }
  }
  return false;
}

template<typename T> bool isonememberof(
  const std::vector<T>& values,
  ::gos::atl::type::range<T>& range) {
  for (std::vector<T>::iterator it = values.begin(); it != values.end(); it++) {
    if (ismemberof(*it, range)) {
      return true;
    }
  }
  return false;
}
#endif

template<typename T> T restrict(
  const T& value,
  const T& minimum,
  const T& maximum){
  if (isinside(value, minimum, maximum)) {
    return value;
  }
 else if (value > maximum) {
    return maximum;
  }
  else {
    return minimum;
  }
}

template<typename T> T restrict(
  const T& value,
  const ::gos::atl::type::range<T>& range){
  return restrict(value, range.lowest, range.highest);
}
}

namespace number {

namespace part {

enum class type { first, second };

template<typename P = uint8_t, typename N = uint16_t>
P first(const N& number) {
  P part;
  memcpy(
    static_cast<void*>(&part),
    static_cast<const void*>(&number),
    sizeof(P));
  return part;
}

template<typename P = uint8_t, typename N = uint16_t>
P second(const N& number) {
  P part;
  memcpy(
    static_cast<void*>(&part),
    static_cast<const void*>(reinterpret_cast<const char*>(&number) + sizeof(P)),
    sizeof(P));
  return part;
}

template<typename P = uint8_t, typename N = uint16_t> void apply(
  N& number,
  const P& part,
  const ::gos::atl::utility::number::part::type& type) {
  switch (type) {
  case ::gos::atl::utility::number::part::type::first:
    memcpy(
      static_cast<void*>(&number),
      static_cast<const void*>(&part),
      sizeof(P));
    break;
  case ::gos::atl::utility::number::part::type::second:
    memcpy(
      static_cast<void*>(reinterpret_cast<char*>(&number) + sizeof(P)),
      static_cast<const void*>(&part),
      sizeof(P));
    break;
  }
}

template<typename P = uint8_t, typename N = uint16_t>
N combine(const P& first, const P& second) {
  N number;
  memcpy(
    static_cast<void*>(&number),
    static_cast<const void*>(&first),
    sizeof(P));
  memcpy(
    static_cast<void*>(reinterpret_cast<char*>(&number) + sizeof(P)),
    static_cast<const void*>(&second),
    sizeof(P));
  return number;
}

}
}

}
}
}

#endif
