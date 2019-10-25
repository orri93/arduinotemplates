#ifndef _GOS_ARDUINO_TEMPLATE_LIBRARY_MODBUS_H_
#define _GOS_ARDUINO_TEMPLATE_LIBRARY_MODBUS_H_

#include <Arduino.h>

#include <ModbusSlave.h>

#include <gatlutility.h>

#define _GOS_ARDUINO_TEMPLATE_LIBRARY_MODBUS_TYPE_SIZE_(x) (sizeof(x)>2?2:1)

namespace gos {
namespace atl {
namespace modbus {

template<typename T> struct Binding {
  uint16_t address;
  T* reference;
};

template<typename T> struct Bindings {
  Binding<T>* bindings;
  uint16_t first;
  uint8_t count;
};

namespace detail {
template<typename T> void next(uint16_t& address) {
  address += _GOS_ARDUINO_TEMPLATE_LIBRARY_MODBUS_TYPE_SIZE_(T);
}
template<typename T> void initialize(
  const Bindings<T>& bindings,
  uint16_t& start,
  uint16_t& lastlength,
  uint8_t& index,
  uint8_t& offset) {
  uint8_t delta = abs(start - bindings.first);
  offset = 0;
  index = 0;
  if (bindings.first > start) {
    lastlength -= delta;
    offset += delta;
    start = bindings.first;
  } else if (bindings.first < start) {
    index += delta;
  }
  lastlength += offset;
}
}

#ifdef GOS_ARDUINO_TEMPLATE_LIBRARY_MODBUS_TESTING
template<typename T> void clean(Bindings<T>& bindings) {
  delete bindings.bindings;
  bindings.bindings = nullptr;
}
template<typename T> uint8_t range(const Bindings<T>& bindings,
  uint16_t start,
  uint16_t lastlength,
  uint8_t& index,
  uint8_t& offset) {
  detail::initialize(bindings, start, lastlength, index, offset);
  uint8_t count = 0;
  uint8_t tin = index;
  uint8_t tof = offset;
  while (tin < bindings.count && tof < lastlength) {
    tof++;
    tin++;
    count++;
  }
  return count;
}
#endif

template<typename T> void create(
  Bindings<T>& bindings,
  uint8_t count,
  uint16_t& address,
  T* reference) {
  bindings.count = count;
  bindings.first = address;
  bindings.bindings = (Binding<T>*)(malloc(sizeof(Binding<T>) * count));
  bindings.bindings->address = address;
  bindings.bindings->reference = reference;
  detail::next<T>(address);
}

template<typename T> void add(
  Bindings<T>& bindings, uint8_t& index, uint16_t& address, T* reference) {
  bindings.bindings[index].address = address;
  bindings.bindings[index].reference = reference;
  detail::next<T>(address);
  index++;
}

namespace coil {
template<typename T> void access(
  Bindings<T>& bindings,
  Modbus& modbus,
  uint16_t start,
  uint16_t lastlength) {
  uint8_t offset, index;
  detail::initialize(bindings, start, lastlength, index, offset);
  while (index < bindings.count && offset < lastlength) {
    modbus.writeCoilToBuffer(offset++, *(bindings.bindings[index++].reference));
  }
}
template<typename T> void assign(
  Bindings<T>& bindings,
  Modbus& modbus,
  uint16_t start,
  uint16_t lastlength) {
  uint8_t offset, index;
  detail::initialize(bindings, start, lastlength, index, offset);
  while (index < bindings.count && offset < lastlength) {
    *(bindings.bindings[index++].reference) =
      modbus.readCoilFromBuffer(offset++);
  }
}
}

namespace registers {
template<typename T> void access(
  Bindings<T>& bindings,
  Modbus& modbus,
  uint16_t start,
  uint16_t lastlength) {
  uint8_t offset, index;
  detail::initialize(bindings, start, lastlength, index, offset);
  while (index < bindings.count && offset < lastlength) {
    modbus.writeRegisterToBuffer(
      offset++,
      *(bindings.bindings[index++].reference));
  }
}
template<typename T> void assign(
  Bindings<T>& bindings,
  Modbus& modbus,
  uint16_t start,
  uint16_t lastlength) {
  uint8_t offset, index;
  detail::initialize(bindings, start, lastlength, index, offset);
  while (index < bindings.count && offset < lastlength) {
    *(bindings.bindings[index++].reference) =
      modbus.readRegisterFromBuffer(offset++);
  }
}
}

namespace discrete {
template<typename T> void access(
  Bindings<T>& bindings,
  Modbus& modbus,
  uint16_t start,
  uint16_t lastlength) {
  uint8_t offset, index;
  detail::initialize(bindings, start, lastlength, index, offset);
  while (index < bindings.count && offset < lastlength) {
    modbus.writeDiscreteInputToBuffer(
      offset++,
      *(bindings.bindings[index++].reference));
  }
}
}

namespace two {
template<typename T> void access(
  Bindings<T>& bindings,
  Modbus& modbus,
  uint16_t start,
  uint16_t lastlength) {
  uint8_t offset, index;
  detail::initialize(bindings, start, lastlength, index, offset);
  while (index < bindings.count && offset < lastlength) {
    modbus.writeRegisterToBuffer(
      offset++, ::gos::atl::utility::number::part::first<uint16_t, T>(
        *(bindings.bindings[index].reference)));
    modbus.writeRegisterToBuffer(
      offset++, ::gos::atl::utility::number::part::second<uint16_t, T>(
        *(bindings.bindings[index].reference)));
    index++;
  }
}
template<typename T> void assign(
  Bindings<T>& bindings,
  Modbus& modbus,
  uint16_t start,
  uint16_t lastlength) {
  uint8_t offset, index;
  detail::initialize(bindings, start, lastlength, index, offset);
  while (index < bindings.count && offset < lastlength) {
    *(bindings.bindings[index].reference) =
      ::gos::atl::utility::number::part::combine<uint16_t, T>(
      modbus.readRegisterFromBuffer(offset),
      modbus.readRegisterFromBuffer(offset + 1)
      );
    offset += 2;
    index++;
  }
}
}


}
}
}

#endif /* _GOS_ARDUINO_TEMPLATE_LIBRARY_MODBUS_H_ */
