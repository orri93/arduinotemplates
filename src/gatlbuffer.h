#ifndef _GOS_ARDUINO_TEMPLATE_LIBRARY_BUFFER_H_
#define _GOS_ARDUINO_TEMPLATE_LIBRARY_BUFFER_H_

#include <Arduino.h>

#include <gatlbinding.h>

#ifndef GOS_ARDUINO_TEMPLATE_LIBRARY_BUFFER_SIZE
#define GOS_ARDUINO_TEMPLATE_LIBRARY_BUFFER_SIZE           11
#endif

namespace gos {
namespace atl {
namespace buffer {

template<typename S = uint8_t, typename C = char> class Holder {
public:
  Holder(const S& size = GOS_ARDUINO_TEMPLATE_LIBRARY_BUFFER_SIZE) : Size(size) {
    Buffer = (C*)(malloc(size));
  }
  Holder(const C* literal, const S& size) : Buffer(const_cast<C*>(literal)), Size(size - 1) {
  }
#ifdef GOS_ARDUINO_TEMPLATE_LIBRARY_CLEANUP
  void cleanup() {
    if (Buffer) {
      free(Buffer);
      Buffer = nullptr;
      Size = S();
    }
  }
#endif
  C* Buffer;
  S Size;
};

template<typename T, typename S = uint8_t, typename C = uint8_t>
S write(Holder<S>& buffer, const S& size, const S& index, T* pointer) {
  C* cp = (C*)(pointer);
  for (S i = 0; i < size; i++) {
    buffer.Buffer[index++] = *(cp++);
  }
  return index;
}

template<typename T, typename S = uint8_t, typename C = uint8_t>
S read(const Holder<S>& buffer, const S& size, const S& index, T* pointer) {
  C* cp = (C*)(pointer);
  for (S i = 0; i < size; i++) {
    *(cp++) = buffer.Buffer[index++];
  }
  return index;
}

template<typename T, typename S = uint8_t, typename C = uint8_t>
S write(
  Holder<S>& buffer,
  ::gos::atl::binding::reference<T, S, C>& binding) {
  S index = binding.first;
  for (uint8_t i = 0; i < binding.count; i++) {
    index = write<T, S, C>(buffer, binding.size, index, binding.pointers[i]);
  }
  return index;
}

template<typename T, typename S = uint8_t, typename C = uint8_t>
S read(
  const Holder<S>& buffer,
  ::gos::atl::binding::reference<T, S, C>& binding) {
  S index = binding.first;
  for (uint8_t i = 0; i < binding.count; i++) {
    index = write<T, S, C>(buffer, binding.size, index, binding.pointers[i]);
  }
  return index;
}

}
}
}

#endif /* _GOS_ARDUINO_TEMPLATE_LIBRARY_BUFFER_H_ */
