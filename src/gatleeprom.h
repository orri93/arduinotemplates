#ifndef _GOS_ARDUINO_TEMPLATE_LIBRARY_EEPROM_H_
#define _GOS_ARDUINO_TEMPLATE_LIBRARY_EEPROM_H_

#include <EEPROM.h>

#include <gatlbinding.h>
#include <gatlutility.h>

#ifdef GOS_ARDUINO_TEMPLATE_LIBRARY_EEPROM_TESTING
#include <cassert>
#endif

namespace gos {
namespace atl {
namespace eeprom {

template<typename T>
int write(const uint8_t& size, int index, T* pointer) {
  uint8_t* cp = (uint8_t*)(pointer);
  for (uint8_t i = 0; i < size; i++) {
    EEPROM.write(index++, *(cp++));
  }
  return index;
}


template<typename T>
int read(const uint8_t& size, int index, T* pointer) {
  uint8_t* cp = (uint8_t*)(pointer);
  for (uint8_t i = 0; i < size; i++) {
    *(cp++) = EEPROM.read(index++);
  }
  return index;
}

template<typename T> void write(
  ::gos::atl::binding::reference<T, int, uint8_t>& binding) {
  int index = binding.first;
  for (uint8_t i = 0; i < binding.count; i++) {
    index = write(binding.size, index, binding.pointers[i]);
  }
}

template<typename T> void read(
  ::gos::atl::binding::reference<T, int, uint8_t>& binding) {
  int index = binding.first;
  for (uint8_t i = 0; i < binding.count; i++) {
    index = read(binding.size, index, binding.pointers[i]);
  }
}

}
}
}

#endif /* _GOS_ARDUINO_TEMPLATE_LIBRARY_EEPROM_H_ */
