#ifndef _GOS_ARDUINO_TEMPLATE_LIBRARY_BINDING_H_
#define _GOS_ARDUINO_TEMPLATE_LIBRARY_BINDING_H_

#include <Arduino.h>

namespace gos {
namespace atl {
namespace binding {

template<typename T, typename A = uint16_t, typename I = uint8_t>
struct reference {
  T** pointers;      // Reference pointer array
  A first;           // First address for the binding
  I count;           // Number of references
  I size;            // Size of each references in either byte or registry count
};

template<typename T, typename R, typename S = uint8_t> S size() {
  return sizeof(T) / sizeof(R);
}

#ifdef GOS_ARDUINO_TEMPLATE_LIBRARY_BINDING_TESTING
namespace testing {
template<typename T, typename A = uint16_t, typename I = uint8_t>
void clean(reference<T,A,I>& reference) {
  free(reference.pointers);
  reference.pointers = nullptr;
}
}
#endif

template<typename T, typename A = uint16_t, typename I = uint8_t>
A create(
  reference<T, A, I>& reference,
  const A& address,
  const I& count,
  const I& size) {
  reference.count = count;
  reference.first = address;
  reference.size = size;
  reference.pointers = (T**)(malloc(sizeof(T*) * count));
  return address + size * count;
}

template<typename T, typename A = uint16_t, typename I = uint8_t>
void set(reference<T, A, I>& reference, const I& index, T* pointer) {
  reference.pointers[index] = pointer;
}

}
}
}

#endif /* _GOS_ARDUINO_TEMPLATE_LIBRARY_BINDING_H_ */
