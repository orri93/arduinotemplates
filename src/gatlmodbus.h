#ifndef _GOS_ARDUINO_TEMPLATE_LIBRARY_MODBUS_H_
#define _GOS_ARDUINO_TEMPLATE_LIBRARY_MODBUS_H_

#include <Arduino.h>

#include <ModbusSlave.h>

#include <gatlbinding.h>
#include <gatlutility.h>

#ifdef GOS_ARDUINO_TEMPLATE_LIBRARY_MODBUS_TESTING
#include <cassert>
#endif

namespace gos {
namespace atl {
namespace modbus {

template<typename T, typename S = uint8_t> S size() {
  return sizeof(T) / 2;
}

namespace detail {
template<typename T> bool initialize(
  // Binding
  const ::gos::atl::binding::reference<T, uint16_t, uint8_t>& reference,
  const uint16_t& start,        // Start address for the modbus function
  const uint16_t& length,       // Length for the modbus function
  uint8_t& offset,              // Offset in the modbus buffer
  uint8_t& from,                // First binding index
  uint8_t& to) {                // Last binding index + 1 covered
  from = 1; to = 0;
  if (reference.first >= start) {
    if (reference.first < (start + length)) {
      //    smmmme
      //    fbbbbbbl
      //
      //    smmmme
      //      fbbbbl
      from = 0;
      offset = reference.first - start;
      to = (start + length - reference.first) / reference.size;
    }
  } else if ((reference.first + reference.count * reference.size) > start) {
    //      smmmme
    //    fbbbbl
    offset = 0;
    from = ((start - reference.first) % reference.size > 0) +
      (start - reference.first) / reference.size;
    to = from + ((start - reference.first) % reference.size == 0) +
      ((reference.first - 1 + (reference.count * reference.size)) - start) /
      reference.size;
  }
  return from < to;
}
}

namespace coil {
template<typename T> void access(
  ::gos::atl::binding::reference<T, uint16_t, uint8_t>& binding,
  Modbus& modbus,
  const uint16_t& startaddress,
  const uint16_t& length) {
  uint8_t offset, from, to;
  detail::initialize(binding, startaddress, length, offset, from, to);
  while (from < to) {
    modbus.writeCoilToBuffer(offset++, *(binding.pointers[from++]));
  }
}
template<typename T> void assign(
  ::gos::atl::binding::reference<T, uint16_t, uint8_t>& binding,
  Modbus& modbus,
  const uint16_t& startaddress,
  const uint16_t& length,
  uint8_t& from,
  uint8_t& to) {
  uint8_t offset;
  detail::initialize(binding, startaddress, length, offset, from, to);
  while (from < to) {
    *(binding.pointers[from++]) = modbus.readCoilFromBuffer(offset++);
  }
}
}

namespace discrete {
template<typename T> void access(
  ::gos::atl::binding::reference<T, uint16_t, uint8_t>& binding,
  Modbus& modbus,
  const uint16_t& startaddress,
  const uint16_t& length) {
  uint8_t offset, from, to;
  detail::initialize(binding, startaddress, length, offset, from, to);
  while (from < to) {
    modbus.writeDiscreteInputToBuffer(offset++, *(binding.pointers[from++]));
  }
}
}

namespace registers {
template<typename T> bool access(
  ::gos::atl::binding::reference<T, uint16_t, uint8_t>& binding,
  Modbus& modbus,
  const uint16_t& startaddress,
  const uint16_t& length) {
  uint8_t offset, from, to;
  bool re = detail::initialize(binding, startaddress, length, offset, from, to);
  while (from < to) {
    modbus.writeRegisterToBuffer(offset++, *(binding.pointers[from++]));
  }
  return re;
}
template<typename T> bool assign(
  ::gos::atl::binding::reference<T, uint16_t, uint8_t>& binding,
  Modbus& modbus,
  const uint16_t& startaddress,
  const uint16_t& length,
  uint8_t& from,
  uint8_t& to) {
  uint8_t offset;
  bool re = detail::initialize(binding, startaddress, length, offset, from, to);
  while (from < to) {
    *(binding.pointers[from++]) = modbus.readRegisterFromBuffer(offset++);
  }
  return re;
}
}

namespace two {
template<typename T> bool access(
  ::gos::atl::binding::reference<T, uint16_t, uint8_t>& binding,
  Modbus& modbus,
  const uint16_t& startaddress,
  const uint16_t& length) {
  uint8_t offset, from, to;
  bool re = detail::initialize(binding, startaddress, length, offset, from, to);
  while (from < to) {
    modbus.writeRegisterToBuffer(
      offset++, ::gos::atl::utility::number::part::first<uint16_t, T>(
        *(binding.pointers[from])));
    modbus.writeRegisterToBuffer(
      offset++, ::gos::atl::utility::number::part::second<uint16_t, T>(
        *(binding.pointers[from])));
    from++;
  }
  return re;
}
template<typename T> bool assign(
  ::gos::atl::binding::reference<T, uint16_t, uint8_t>& binding,
  Modbus& modbus,
  const uint16_t& startaddress,
  const uint16_t& length,
  uint8_t& from,
  uint8_t& to) {
  uint8_t offset, index;
  bool re = detail::initialize(binding, startaddress, length, offset, from, to);
  index = from;
  while (index < to) {
    *(binding.pointers[index]) =
      ::gos::atl::utility::number::part::combine<uint16_t, T>(
      modbus.readRegisterFromBuffer(offset),
      modbus.readRegisterFromBuffer(offset + 1)
      );
    offset += 2;
    index++;
  }
  return re;
}
}

}
}
}

#endif /* _GOS_ARDUINO_TEMPLATE_LIBRARY_MODBUS_H_ */
