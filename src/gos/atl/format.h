#ifndef _GOS_ARDUINO_TEMPLATE_LIBRARY_FORMAT_H_
#define _GOS_ARDUINO_TEMPLATE_LIBRARY_FORMAT_H_

#include <Arduino.h>

#include <gos/atl/buffer.h>

#define GOS_ARDUINO_TEMPLATE_LIBRARY_WIDTH_FILL           127
#define GOS_ARDUINO_TEMPLATE_LIBRARY_WIDTH_FILL_NEGATIVE -127
#define GOS_ARDUINO_TEMPLATE_LIBRARY_WIDTH_UNDEFINED        0

#ifndef GOS_ARDUINO_TEMPLATE_LIBRARY_WIDTH
#define GOS_ARDUINO_TEMPLATE_LIBRARY_WIDTH GOS_ARDUINO_TEMPLATE_LIBRARY_WIDTH_FILL
#endif

#ifndef GOS_ARDUINO_TEMPLATE_LIBRARY_PRECISION
#define GOS_ARDUINO_TEMPLATE_LIBRARY_PRECISION              1
#endif

namespace gos {
namespace atl {

namespace format {
namespace option {
struct Real {
  Real(
    const int8_t& width = GOS_ARDUINO_TEMPLATE_LIBRARY_WIDTH,
    const uint8_t& precision = GOS_ARDUINO_TEMPLATE_LIBRARY_PRECISION) :
    Width(width),
    Precision(precision) {
  }
  int8_t Width;
  uint8_t Precision;
};
typedef struct Real Number;
}

#ifdef GOS_ARDUINO_TEMPLATE_LIBRARY_CHECK_SIZE
namespace check {
template<typename S = uint8_t>
inline void fix(
  int& check,
  const ::gos::atl::buffer::Holder<S>* prefix = nullptr,
  const ::gos::atl::buffer::Holder<S>* postfix = nullptr) {
  if (prefix) {
    check -= prefix->Size;
  }
  if (postfix) {
    check -= postfix->Size;
  }
}
inline bool option(
  int& check,
  const ::gos::atl::format::option::Number& option) {
  switch (option.Width) {
  case GOS_ARDUINO_TEMPLATE_LIBRARY_WIDTH_FILL:
  case GOS_ARDUINO_TEMPLATE_LIBRARY_WIDTH_FILL_NEGATIVE:
    if (check < 3) {
      return false;
    }
    break;
  default:
    check -= abs(option.Width);
  }
  if (check < 0) {
    return false;
  }
  return true;
}
template<typename S = uint8_t>
inline bool real(
  buffer::Holder<S>& holder,
  const ::gos::atl::format::option::Number& option,
  const ::gos::atl::buffer::Holder<S>* prefix = nullptr,
  const ::gos::atl::buffer::Holder<S>* postfix = nullptr) {
  int check = static_cast<int>(holder.Size - 1);
  fix(check, prefix, postfix);
  return ::gos::atl::format::check::option(check, option);
}
template<typename T, typename S = uint8_t>
inline bool integer(
  buffer::Holder<S>& holder,
  const T& value,
  const ::gos::atl::buffer::Holder<S>* prefix = nullptr,
  const ::gos::atl::buffer::Holder<S>* postfix = nullptr,
  int8_t width = GOS_ARDUINO_TEMPLATE_LIBRARY_WIDTH_UNDEFINED) {
  int check = static_cast<int>(holder.Size - 1);
  fix(check, prefix, postfix);
  int length = snprintf(holder.Buffer, holder.Size, "%d", value);
  switch (width) {
  case GOS_ARDUINO_TEMPLATE_LIBRARY_WIDTH_UNDEFINED:
  case GOS_ARDUINO_TEMPLATE_LIBRARY_WIDTH_FILL_NEGATIVE:
  case GOS_ARDUINO_TEMPLATE_LIBRARY_WIDTH_FILL:
    return length <= check;
  default:
    return check >= width;
  }
}
}
#endif

template<typename S = uint8_t> void fix(
  S& size,
  char** pointer,
  const ::gos::atl::buffer::Holder<S>* prefix = nullptr,
  const ::gos::atl::buffer::Holder<S>* postfix = nullptr) {
  if (prefix) {
    memcpy(*pointer, prefix->Buffer, prefix->Size);
    *pointer += prefix->Size;
    size -= prefix->Size;
  }
  if (postfix) {
    size -= postfix->Size;
  }
}

template<typename S = uint8_t> void postfix(
  char** pointer,
  const ::gos::atl::buffer::Holder<S>* postfix = nullptr) {
  if (postfix) {
    memcpy(*pointer, postfix->Buffer, postfix->Size);
    *pointer += postfix->Size;
  }
}

template<typename T, typename S = uint8_t> void real(
  buffer::Holder<S>& holder,
  const T& value,
  const ::gos::atl::format::option::Number& option,
  const ::gos::atl::buffer::Holder<S>* prefix = nullptr,
  const ::gos::atl::buffer::Holder<S>* postfix = nullptr) {
  S size = holder.Size - 1;       /* Terminating zero */
  char *pointer = holder.Buffer;
  fix(size, &pointer, prefix, postfix);
  int8_t width = option.Width;
  switch (option.Width) {
  case GOS_ARDUINO_TEMPLATE_LIBRARY_WIDTH_FILL:
    width = size;
    break;
  case GOS_ARDUINO_TEMPLATE_LIBRARY_WIDTH_FILL_NEGATIVE:
    width = -size;
    break;
  }
  ::dtostrf(static_cast<double>(value), width, option.Precision, pointer);
  pointer += abs(width);
  ::gos::atl::format::postfix(&pointer, postfix);
  *pointer = '\0';
}

template<typename T, typename S = uint8_t> void integer(
  buffer::Holder<S>& holder,
  const T& value,
  const ::gos::atl::buffer::Holder<S>* prefix = nullptr,
  const ::gos::atl::buffer::Holder<S>* postfix = nullptr,
  int8_t width = GOS_ARDUINO_TEMPLATE_LIBRARY_WIDTH) {
  S size = holder.Size - 1;       /* Terminating zero */
  char *pointer = holder.Buffer;
  fix(size, &pointer, prefix, postfix);
#ifdef GOS_ARDUINO_UNIT_TESTING
  S length = snprintf(pointer, size, "%d", value);
#else
  S length sprintf(pointer, "%d", value);
#endif
  switch (width)
  {
  case GOS_ARDUINO_TEMPLATE_LIBRARY_WIDTH_UNDEFINED:
    pointer += length;
    break;
  case GOS_ARDUINO_TEMPLATE_LIBRARY_WIDTH_FILL_NEGATIVE:
    memset(pointer + length, ' ', size - length);
    pointer += size;
    break;
  case GOS_ARDUINO_TEMPLATE_LIBRARY_WIDTH_FILL:
    width = size;
  default:
    if (size - length > 0) {
      memmove(pointer + width - length, pointer, length);
      memset(pointer, ' ', width - length);
    }
    pointer += width;
    break;
  }
  ::gos::atl::format::postfix(&pointer, postfix);
  *pointer = '\0';
}

template<typename S = uint8_t> void error(
  buffer::Holder<S>& holder,
  const char* message,
  const uint8_t& length,
  const ::gos::atl::buffer::Holder<S>* prefix = nullptr) {
  S size = holder.Size;             /* Terminating zero */
  char *pointer = holder.Buffer;
  fix(size, &pointer, prefix);
  size = min(size, length);
  memcpy(pointer, message, size);
}

}

}
}

#endif /* _GOS_ARDUINO_TEMPLATE_LIBRARY_FORMAT_H_ */
