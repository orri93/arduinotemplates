#ifndef _GOS_ARDUINO_TEMPLATE_LIBRARY_STRING_H_
#define _GOS_ARDUINO_TEMPLATE_LIBRARY_STRING_H_

#include <Arduino.h>

#include <gatlbuffer.h>

namespace gos {
namespace atl {

namespace string {

template<typename T, typename S = uint8_t, typename C = uint8_t>
void copy(Holder<S>& buffer, const T* str) {
  ::strncpy(buffer.Buffer, str, buffer.Size);
}

template<typename T, typename S = uint8_t, typename C = uint8_t>
int compare(Holder<S>& buffer, const T* str) {
  return ::strncmp(buffer.Buffer, str, buffer.Size);
}

}

}
}

#endif /* _GOS_ARDUINO_TEMPLATE_LIBRARY_FORMAT_H_ */
