#ifndef _GOS_ARDUINO_TEMPLATE_LIBRARY_STRING_H_
#define _GOS_ARDUINO_TEMPLATE_LIBRARY_STRING_H_

#include <Arduino.h>

#include <gatlbuffer.h>

#define GATL_TEXT_MEMCPY(d,x) memcpy(d, x, sizeof(x)) 

namespace gos {
namespace atl {

namespace string {

template<typename S = uint8_t, typename C = char>
void copy(::gos::atl::buffer::Holder<S, C>& holder, const char* str) {
  ::strncpy((char*)(holder.Buffer), str, holder.Size);
}

template<typename S = uint8_t, typename C = char>
int compare(const ::gos::atl::buffer::Holder<S, C>& holder, const char* str) {
  return ::strncmp((const char*)(holder.Buffer), str, holder.Size);
}

} // namespace string

} // namespace atl
} // namespace gos

#endif /* _GOS_ARDUINO_TEMPLATE_LIBRARY_STRING_H_ */
