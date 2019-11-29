#ifndef _GOS_ARDUINO_TEMPLATE_LIBRARY_BINDING_H_
#define _GOS_ARDUINO_TEMPLATE_LIBRARY_BINDING_H_

#include <Arduino.h>

namespace gos {
namespace atl {
namespace binding {

template<typename T, typename R, typename S = uint8_t> S size() {
  return sizeof(T) / sizeof(R);
}

template<typename T, typename A = uint16_t, typename I = uint8_t>
struct reference {
  T** pointers;      // Reference pointer array
  A first;           // First address for the binding
  I count;           // Number of references
  I size;            // Size of each references in either byte or registry count
};


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

#ifdef GOS_ARDUINO_TEMPLATE_LIBRARY_BINDING_TESTING
namespace testing {
template<typename T, typename A = uint16_t, typename I = uint8_t>
void clean(reference<T, A, I>& reference) {
  free(reference.pointers);
  reference.pointers = nullptr;
}
}
#endif

namespace change {
namespace aware {
template<typename T, typename A = uint16_t, typename I = uint8_t>
struct reference : ::gos::atl::binding::reference<T, A, I> {
  uint8_t* status;   // Status bit array
};

template<typename T, typename A = uint16_t, typename I = uint8_t>
A create(
  reference<T, A, I>& reference,
  const A& address,
  const I& count,
  const I& size) {
  reference.status = (uint8_t*)(malloc(1 + count / 8));
  ::memset(reference.status, 0x00, 1 + count / 8);
  return ::gos::atl::binding::create<T, A, I>(reference, address, count, size);
}

template<typename T, typename A = uint16_t, typename I = uint8_t>
void set(reference<T, A, I>& reference, const I& index, const bool& changed) {
  if (changed) {
    bitSet(reference.status[index / 8], index % 8);
  } else {
    bitClear(reference.status[index / 8], index % 8);
  }
}

#ifdef GOS_ARDUINO_TEMPLATE_LIBRARY_BINDING_TESTING
namespace testing {
template<typename T, typename A = uint16_t, typename I = uint8_t>
void clean(reference<T, A, I>& reference) {
  ::gos::atl::binding::testing::clean(reference);
  free(reference.status);
  reference.status = nullptr;
}
}
#endif
}
template<typename T, typename A = uint16_t, typename I = uint8_t>
bool is(
  ::gos::atl::binding::change::aware::reference<T, A, I>& reference,
  const I& index) {
  return bitRead(reference.status[index / 8], index % 8);
}
}



}
}
}

#endif /* _GOS_ARDUINO_TEMPLATE_LIBRARY_BINDING_H_ */
