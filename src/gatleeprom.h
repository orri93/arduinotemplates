#ifndef _GOS_ARDUINO_TEMPLATE_LIBRARY_EEPROM_H_
#define _GOS_ARDUINO_TEMPLATE_LIBRARY_EEPROM_H_

#ifdef GOS_ATL_USE_AVR_EEPROM
#include <avr/eeprom.h>
#else
#include <EEPROM.h>
#endif

#include <gatlbuffer.h>
#include <gatlbinding.h>
#include <gatlutility.h>

#ifdef GOS_ARDUINO_TEMPLATE_LIBRARY_EEPROM_TESTING
#include <cassert>
#endif

namespace gos {
namespace atl {
namespace eeprom {

template<typename T, typename A>
A write(const uint8_t& size, A address, T* pointer) {
  uint8_t* cp = (uint8_t*)(pointer);
  for (uint8_t i = 0; i < size; i++) {
#ifdef GOS_ATL_USE_AVR_EEPROM
    ::eeprom_write_byte(reinterpret_cast<uint8_t*>(address++), *(cp++));
#else
    EEPROM.write(address++, *(cp++));
#endif
  }
  return address;
}

template<typename T, typename A>
A update(const uint8_t& size, A address, T* pointer) {
  uint8_t* cp = (uint8_t*)(pointer);
  for (uint8_t i = 0; i < size; i++) {
#ifdef GOS_ATL_USE_AVR_EEPROM
    ::eeprom_update_byte((uint8_t*)address++, *(cp++));
#else
    EEPROM.update(address++, *(cp++));
#endif
  }
  return address;
}

template<typename T>
int read(const uint8_t& size, int index, T* pointer) {
  uint8_t* cp = (uint8_t*)(pointer);
  for (uint8_t i = 0; i < size; i++) {
#ifdef GOS_ATL_USE_AVR_EEPROM
    * (cp++) = ::eeprom_read_byte(reinterpret_cast<const uint8_t*>( index++));
#else
    *(cp++) = EEPROM.read(index++);
#endif
  }
  return index;
}

#ifdef GOS_ATL_USE_AVR_EEPROM
template<typename S = uint8_t, typename A = uint16_t>
void read(
  ::gos::atl::buffer::Holder<S>& buffer,
  const A& address,
  const S& size) {
  ::eeprom_read_block(
    (void*)(buffer.Buffer),
    (const void*)address,
    (size_t)size);
}
template<typename S = uint8_t, typename A = uint16_t>
void update(
  const ::gos::atl::buffer::Holder<S>& buffer,
  const A& address,
  const S& size) {
  ::eeprom_update_block(
    (const void*)(buffer.Buffer),
    (void*)address,
    (size_t)size);
}
template<typename S = uint8_t, typename A = uint16_t>
void write(
  const ::gos::atl::buffer::Holder<S>& buffer,
  const A& address,
  const S& size) {
  ::eeprom_write_block(
    (const void*)(buffer.Buffer),
    (void*)address,
    (size_t)size);
}
#endif

template<typename T> void write(
  ::gos::atl::binding::reference<T, int, uint8_t>& binding) {
  int index = binding.first;
  for (uint8_t i = 0; i < binding.count; i++) {
    index = write(binding.size, index, binding.pointers[i]);
  }
}

template<typename T> void update(
  ::gos::atl::binding::reference<T, int, uint8_t>& binding) {
  int index = binding.first;
  for (uint8_t i = 0; i < binding.count; i++) {
    index = update(binding.size, index, binding.pointers[i]);
  }
}

template<typename T> void read(
  ::gos::atl::binding::reference<T, int, uint8_t>& binding) {
  int index = binding.first;
  for (uint8_t i = 0; i < binding.count; i++) {
    index = read(binding.size, index, binding.pointers[i]);
  }
}

template<typename T> void write(
  ::gos::atl::binding::reference<T, int, uint8_t>& binding,
  const int& index) {
  write(binding.size, binding.first + index, binding.pointers[index]);
}

template<typename T> void update(
  ::gos::atl::binding::reference<T, int, uint8_t>& binding,
  const int& index) {
  update(binding.size, binding.first + index, binding.pointers[index]);
}

template<typename T> void read(
  ::gos::atl::binding::reference<T, int, uint8_t>& binding,
  const int& index) {
  read(binding.size, binding.first + index, binding.pointers[index]);
}



template<typename T, typename A, typename C> void write(
  ::gos::atl::binding::barray::reference<T, A, C>& binding) {
  int index = binding.first;
  for (uint8_t i = 0; i < binding.count; i++) {
    index = write(binding.size, index, binding.pointers + i);
  }
}

template<typename T, typename A, typename C> void update(
  ::gos::atl::binding::barray::reference<T, A, C>& binding) {
  int index = binding.first;
  for (uint8_t i = 0; i < binding.count; i++) {
    index = update(binding.size, index, binding.pointers + i);
  }
}

template<typename T, typename A, typename C> void read(
  ::gos::atl::binding::barray::reference<T, A, C>& binding) {
  int index = binding.first;
  for (uint8_t i = 0; i < binding.count; i++) {
    index = read(binding.size, index, binding.pointers + i);
  }
}

template<typename T, typename A, typename C> void write(
  ::gos::atl::binding::barray::reference<T, A, C>& binding,
  const A& index) {
  write(binding.size, binding.first + index, binding.pointers + index);
}

template<typename T, typename A, typename C> void update(
  ::gos::atl::binding::barray::reference<T, A, C>& binding,
  const int& index) {
  update(binding.size, binding.first + index, binding.pointers + index);
}

template<typename T, typename A, typename C > void read(
  ::gos::atl::binding::barray::reference<T, A, C>& binding,
  const int& index) {
  read(binding.size, binding.first + index, binding.pointers + index);
}

}
}
}

#endif /* _GOS_ARDUINO_TEMPLATE_LIBRARY_EEPROM_H_ */
