#ifndef _GOS_ARDUINO_TEMPLATE_LIBRARY_MODBUS_H_
#define _GOS_ARDUINO_TEMPLATE_LIBRARY_MODBUS_H_

#include <Arduino.h>

#if USE_ARDUINO_MODBUS_SLAVE
#include <ModbusSlave.h>
#endif

#include <gatlbinding.h>
#include <gatlutility.h>
#include <gatlbuffer.h>

#ifdef GOS_ARDUINO_TEMPLATE_LIBRARY_MODBUS_TESTING
#include <cassert>
#endif

/*
 * Example
 *   Binding A from address 0x0000 to 0x0002 each only one register
 *   Binding B from address 0x0003 to 0x0008 each two register
 *
 *   Modbus buffer is of length size and always starts from 0
 *
 * Calculation rules
 *     binding end = binding start + binding count * binding size
 *     modbus end address = modbus start address + length - 1
 *   For binding to be included the following must hold
 *     modbus start address + modbus length >= binding start
 *     modbus start address <= binding end
 *   If the modbus start address >= binding start
 *       mmmm          mmmm      mmmmmm        mmmm
 *       bbbb   or   bbbb   or   bbbb   or   bbbbbbbb
 *     the first address to consider is always modbus start address
 *     the first item in the modbus buffer is always 0
 *     the first in the binding to consider is
 *       first item index = (modbus start address - binding start) / binding size
 *     then to check if address is correct
 *       (((modbus start address - binding start) % binding size) == 0)
 *     then the items can be read one by one until either
 *       reach the end of register in the binding or the number of addresses or over
 *         buffer index = 0
 *         index = first item index
 *         buffer count = length
 *         if modbus end address > binding end {
 *           buffer count -= modbus end address - binding end
 *         }
 *         while(buffer index < buffer count && index < binding count) {
 *           access/assign
 *           buffer index += binding size;
 *           bind index++;
 *         }
 *   else the modbus start address < binding start
 *     mmmm        mmmmmm      mmmmmmmm
 *       bbbb   or   bbbb   or   bbbb
 *     the first in the binding to consider is always 0
 *     the first item in the modbus buffer to consider is binding start - modbus start address
 *     the first modbus address to consider is the binding start address
 *     to check if the address is correct
 *       if(binding end >  modbus end address) {
 *         (((binding end - modbus end address) % binding size == 0)
 *     the items can be read one by one until either
 *       reach the last modbus address or the last binding index
 *         binding index = 0
 *         buffer index = binding start - modbus start address
 *         address = binding start
 *         while(buffer index < buffer length && index < binding count) {
 *           access/assign
 *           buffer index += binding size;
 *           bind index++;
 *         }
 *
 *     initialize input is the binding, modbus start address, modbus buffer length
 *                output is first binding index, first buffer index and buffer count
 *
 */

#define MODBUS_INVALID_UNIT_ADDRESS 255
#define MODBUS_DEFAULT_UNIT_ADDRESS 1
#define MODBUS_CONTROL_PIN_NONE -1

 /**
  * Modbus function codes
  */
enum {
  MODBUS_FC_INVALID = 0,
  MODBUS_FC_READ_COILS = 1,
  MODBUS_FC_READ_DISCRETE_INPUT = 2,
  MODBUS_FC_READ_HOLDING_REGISTERS = 3,
  MODBUS_FC_READ_INPUT_REGISTERS = 4,
  MODBUS_FC_WRITE_COIL = 5,
  MODBUS_FC_WRITE_REGISTER = 6,
  MODBUS_FC_READ_EXCEPTION_STATUS = 7,
  MODBUS_FC_WRITE_MULTIPLE_COILS = 15,
  MODBUS_FC_WRITE_MULTIPLE_REGISTERS = 16
};

enum {
  MODBUS_CB_MIN = 0,
  MODBUS_CB_READ_COILS = MODBUS_CB_MIN,
  MODBUS_CB_READ_DISCRETE_INPUTS,
  MODBUS_CB_READ_HOLDING_REGISTERS,
  MODBUS_CB_READ_INPUT_REGISTERS,
  MODBUS_CB_WRITE_COILS,
  MODBUS_CB_WRITE_HOLDING_REGISTERS,
  MODBUS_CB_READ_EXCEPTION_STATUS,
  MODBUS_CB_MAX
};

enum {
  MODBUS_COIL_OFF = 0x0000,
  MODBUS_COIL_ON = 0xff00
};

enum {
  MODBUS_STATUS_OK = 0,
  MODBUS_STATUS_ILLEGAL_FUNCTION,
  MODBUS_STATUS_ILLEGAL_DATA_ADDRESS,
  MODBUS_STATUS_ILLEGAL_DATA_VALUE,
  MODBUS_STATUS_SLAVE_DEVICE_FAILURE,
  MODBUS_STATUS_ACKNOWLEDGE,
  MODBUS_STATUS_SLAVE_DEVICE_BUSY,
  MODBUS_STATUS_NEGATIVE_ACKNOWLEDGE,
  MODBUS_STATUS_MEMORY_PARITY_ERROR,
  MODBUS_STATUS_GATEWAY_PATH_UNAVAILABLE,
  MODBUS_STATUS_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND,
};

#define MODBUS_FRAME_SIZE 4
#define MODBUS_CRC_LENGTH 2

#define MODBUS_ADDRESS_INDEX 0
#define MODBUS_FUNCTION_CODE_INDEX 1
#define MODBUS_DATA_INDEX 2

#define MODBUS_BROADCAST_ADDRESS 0
#define MODBUS_ADDRESS_MIN 1
#define MODBUS_ADDRESS_MAX 247

#define MODBUS_HALF_SILENCE_MULTIPLIER 3
#define MODBUS_FULL_SILENCE_MULTIPLIER 7

#define MODBUS_READ_UINT16(arr, index) word(arr[index], arr[index + 1])
#define MODBUS_READ_READCRC(arr, length) word(arr[(length - MODBUS_CRC_LENGTH) \
  + 1], arr[length - MODBUS_CRC_LENGTH])

namespace gos {
namespace atl {
namespace modbus {

namespace binding {
enum class result { undefined, included, excluded, failure };

template<typename T, typename S = uint8_t> S size() {
  return sizeof(T) / 2;
}

namespace detail {
template<typename T>
result initializenew(
  const ::gos::atl::binding::reference<T, uint16_t, uint8_t>& reference,
  const uint16_t& start,       // Start address for the modbus function
  const uint16_t& length,      // Length of the buffer for the modbus function
  uint16_t& address,           // The first modbus address that is affected
  uint16_t& first,             // The first buffer item to consider
  uint16_t& last,              // The last buffer item to consider
  uint8_t& index) {            // The first index in the binding to consider
  if (start > reference.first) {
    if (((start - reference.first) % reference.size) == 0) {
      /* Using address to hold the difference temporarily */
      address = (start + length) -
        (reference.first + reference.count * reference.size);
      if (address > 0) {
        if ((address / reference.size) < reference.count) {
          last = length - address;
          address = start;
          first = 0;
          index = start - reference.first / reference.size;
          return result::included;
        } else {
          return result::excluded;
        }
      }
    } else {
      return result::failure;
    }
  } else {
    /* Using address to hold the difference temporarily */
    address = (reference.first + reference.count * reference.size) -
      (start + length);
    if (address > 0) {
      if ((address % reference.size) == 0) {
        address = reference.first;
        first = reference.first - start;
        index = 0;
        last = length;
        return result::included;
      } else {
        return result::failure;
      }
    } else {
      return result::excluded;
    }
  }
}
template<typename T> int initialize(
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
} // detail namespace

#if USE_ARDUINO_MODBUS_SLAVE
namespace coil {
template<typename T> bool access(
  ::gos::atl::binding::reference<T, uint16_t, uint8_t>& binding,
  Modbus& modbus,
  const uint16_t& startaddress,
  const uint16_t& length) {
  uint8_t offset, from, to;
  bool re = detail::initialize(binding, startaddress, length, offset, from, to);
  while (from < to) {
    modbus.writeCoilToBuffer(offset++, *(binding.pointers[from++]));
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
    *(binding.pointers[index++]) = modbus.readCoilFromBuffer(offset++);
  }
  return re;
}
template<typename T> bool assign(
  ::gos::atl::binding::change::aware::reference<T, uint16_t, uint8_t>& binding,
  Modbus& modbus,
  const uint16_t& startaddress,
  const uint16_t& length,
  uint8_t& from,
  uint8_t& to) {
  T value;
  uint8_t offset, index;
  bool result = detail::initialize(binding, startaddress, length, offset, from, to);
  index = from;
  while (index < to) {
    value = modbus.readCoilFromBuffer(offset++);
    if (value != *(binding.pointers[index])) {
      *(binding.pointers[index]) = value;
      ::gos::atl::binding::change::aware::set<T, uint16_t, uint8_t>(
        binding, index, true);
    } else {
      ::gos::atl::binding::change::aware::set<T, uint16_t, uint8_t>(
        binding, index, false);
    }
    index++;
  }
  return result;
}

} // coil namespace

namespace discrete {
template<typename T> bool access(
  ::gos::atl::binding::reference<T, uint16_t, uint8_t>& binding,
  Modbus& modbus,
  const uint16_t& startaddress,
  const uint16_t& length) {
  uint8_t offset, from, to;
  bool re = detail::initialize(binding, startaddress, length, offset, from, to);
  while (from < to) {
    modbus.writeDiscreteInputToBuffer(offset++, *(binding.pointers[from++]));
  }
  return re;
}
} // descrete namespace

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
  uint8_t offset, index;
  bool re = detail::initialize(binding, startaddress, length, offset, from, to);
  index = from;
  while (index < to) {
    *(binding.pointers[index++]) = modbus.readRegisterFromBuffer(offset++);
  }
  return re;
}
template<typename T> bool assign(
  ::gos::atl::binding::change::aware::reference<T, uint16_t, uint8_t>& binding,
  Modbus& modbus,
  const uint16_t& startaddress,
  const uint16_t& length,
  uint8_t& from,
  uint8_t& to) {
  T value;
  uint8_t offset, index;
  bool result = detail::initialize(binding, startaddress, length, offset, from, to);
  index = from;
  while (index < to) {
    value = modbus.readRegisterFromBuffer(offset++);
    if (value != *(binding.pointers[index])) {
      *(binding.pointers[index]) = value;
      ::gos::atl::binding::change::aware::set<T, uint16_t, uint8_t>(
        binding, index, true);
    } else {
      ::gos::atl::binding::change::aware::set<T, uint16_t, uint8_t>(
        binding, index, false);
    }
    index++;
  }
  return result;
}
} // registers namespace

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
template<typename T> bool assign(
  ::gos::atl::binding::change::aware::reference<T, uint16_t, uint8_t>& binding,
  Modbus& modbus,
  const uint16_t& startaddress,
  const uint16_t& length,
  uint8_t& from,
  uint8_t& to) {
  uint8_t offset, index;
  bool result = detail::initialize(
    binding,
    startaddress,
    length,
    offset,
    from,
    to);
  index = from;
  T value;
  while (index < to && offset < length) {
    value = ::gos::atl::utility::number::part::combine<uint16_t, T>(
      modbus.readRegisterFromBuffer(offset),
      modbus.readRegisterFromBuffer(offset + 1)
      );
    if (value != *(binding.pointers[index])) {
      *(binding.pointers[index]) = value;
      ::gos::atl::binding::change::aware::set<T, uint16_t, uint8_t>(
        binding, index, true);
    } else {
      ::gos::atl::binding::change::aware::set<T, uint16_t, uint8_t>(
        binding, index, false);
    }
    offset += 2;
    index++;
  }
  to = index;
  return result;
}
} // two namespace
#endif
} // binding namespace

template<
  typename A = uint16_t,
  typename F = uint16_t,
  typename L = uint16_t,
  typename R = uint8_t>
  class Handler {
  public:
    typedef A Address;
    typedef F Function;
    typedef L Length;
    typedef R Result;
    virtual ~Handler() {}
    virtual Result ReadCoils(
      const Function& function,
      const Address& address,
      const Length& length) {
      return MODBUS_STATUS_ILLEGAL_FUNCTION;
    }
    virtual Result ReadHoldingRegisters(
      const Function& function,
      const Address& address,
      const Length& length) {
      return MODBUS_STATUS_ILLEGAL_FUNCTION;
    }
    virtual Result ReadInputRegisters(
      const Function& function,
      const Address& address,
      const Length& length) {
      return MODBUS_STATUS_ILLEGAL_FUNCTION;
    }
    virtual Result WriteCoils(
      const Function& function,
      const Address& address,
      const Length& length) {
      return MODBUS_STATUS_ILLEGAL_FUNCTION;
    }
    virtual Result WriteHoldingRegisters(
      const Function& function,
      const Address& address,
      const Length& length) {
      return MODBUS_STATUS_ILLEGAL_FUNCTION;
    }
    virtual Result ReadExceptionStatus(const Function& function) {
      return MODBUS_STATUS_ILLEGAL_FUNCTION;
    }
};

namespace structures {
template<typename A = uint16_t, typename P = int>
struct Parameter {

  typedef A address;
  typedef P pin;
  address SlaveId;
  pin TransmissionControl;
};

template<typename I = uint16_t>
struct Index {
  typedef I index;
  index ResponseWrite;
};
template<typename L = uint16_t>
struct Length {
  typedef L length;
  length TotalSent;
  length TotalReceived;
  length TransmissionBuffer;
  length RequestBuffer;
  length ResponseBuffer;
};

template<typename Bit = bool>
struct State {
  typedef Bit state;
  state ResponseBufferWriting;
  state RequestBufferReading;
};

template<typename M = uint16_t, typename T = uint64_t>
struct Time {
  typedef M microsecond;
  typedef T time;
  microsecond HalfChar;
  time LastCommunication;
};

template<
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename T = uint64_t>
  struct Variable {
  struct Time<M, T> Time;
  struct Length<L> Length;
  struct Index<I> Index;
  struct State<> Is;
};
}

namespace details {

namespace write {
/**
 * Writes the output buffer to serial stream
 *
 * @return The number of bytes written
 */
template<
  typename A = uint16_t,
  typename C = uint16_t,
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename P = int,
  typename S = uint16_t,
  typename T = uint64_t>
  L response(
    Stream& stream,
    const ::gos::atl::modbus::structures::Parameter<A, P> parameter,
    ::gos::atl::modbus::structures::Variable<I, L, M, T>& variable,
    ::gos::atl::buffer::Holder<S, char>& response) {
  /**
  * Validate
  */
  // check if there is a response and this is supposed to be the first write
  if (variable.Index.ResponseWrite == 0 && variable.Length.ResponseBuffer >= MODBUS_FRAME_SIZE) {
    // set status as writing
    variable.Is.ResponseBufferWriting = true;
  }

  // check if we are not in writing or the address is broadcast
  if (!variable.Is.ResponseBufferWriting ||
    response.Buffer[MODBUS_ADDRESS_INDEX] == MODBUS_BROADCAST_ADDRESS) {
    // cleanup and ignore
    variable.Is.ResponseBufferWriting = false;
    variable.Index.ResponseWrite = 0;
    variable.Length.ResponseBuffer = 0;
    return 0;
  }

  /**
   * Preparing
   */

   // if this is supposed to be the first write
  if (variable.Index.ResponseWrite == 0) {
    // if we still need to wait
    if ((micros() - variable.Time.LastCommunication) <=
      (variable.Time.HalfChar * MODBUS_HALF_SILENCE_MULTIPLIER))
    {
      // ignore
      return 0;
    }

    // calculate and fill crc
    C crc = ::gos::atl::utility::crc::calculate<C, I, S>(
      response,
      variable.Length.ResponseBuffer - MODBUS_CRC_LENGTH);
    response.Buffer[variable.Length.ResponseBuffer - MODBUS_CRC_LENGTH] =
      crc & 0xff;
    response.Buffer[(variable.Length.ResponseBuffer - MODBUS_CRC_LENGTH) + 1] =
      crc >> 8;

    // enter transmission mode
    if (parameter.TransmissionControl > MODBUS_CONTROL_PIN_NONE) {
      digitalWrite(parameter.TransmissionControl, HIGH);
    }
  }

  /**
   * Transmit
   */

   // send buffer
  L length = 0;
  if (variable.Length.TransmissionBuffer > 0) {
    L length = min(
      stream.availableForWrite(),
      variable.Length.ResponseBuffer - variable.Index.ResponseWrite);

    if (length > 0) {
      length = (L)(stream.write(
        (uint8_t*)(response.Buffer + variable.Index.ResponseWrite),
        static_cast<size_t>(length)));
      variable.Index.ResponseWrite += length;
      variable.Length.TotalSent += length;
    }

    if (stream.availableForWrite() < variable.Length.TransmissionBuffer)
    {
      // still waiting for write to complete
      variable.Time.LastCommunication = micros();
      return length;
    }

    // if buffer reports as empty; make sure it really is 
    // (`Serial` removes bytes from buffer before sending them)
    stream.flush();
  } else {
    // compatibility for badly written software serials; aka AltSoftSerial
    length = variable.Length.ResponseBuffer - variable.Index.ResponseWrite;

    if (length > 0) {
      length = (L)(stream.write(
        (const uint8_t*)(response.Buffer),
        static_cast<size_t>(length)));
      stream.flush();
    }

    variable.Index.ResponseWrite += length;
    variable.Length.TotalSent += length;
  }

  if (variable.Index.ResponseWrite >= variable.Length.ResponseBuffer &&
    (micros() - variable.Time.LastCommunication) >
    (variable.Time.HalfChar * MODBUS_HALF_SILENCE_MULTIPLIER)) {

    // end transmission
    if (parameter.TransmissionControl > MODBUS_CONTROL_PIN_NONE) {
      digitalWrite(parameter.TransmissionControl, LOW);
    }

    // cleanup
    variable.Is.ResponseBufferWriting = false;
    variable.Index.ResponseWrite = 0;
    variable.Length.ResponseBuffer = 0;
  }

  return length;
}
}

namespace read {
template<
  typename A = uint16_t,
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename P = int,
  typename S = uint16_t,
  typename T = uint64_t>
  bool request(
    Stream& stream,
    const ::gos::atl::modbus::structures::Parameter<A, P> parameter,
    ::gos::atl::modbus::structures::Variable<I, L, M, T>& variable,
    ::gos::atl::buffer::Holder<S, char>& request) {
  /**
  * Read one data packet and report when received completely
  */
  L length = (L)(stream.available());

  if (length > L()) {
    // if not yet started reading
    if (!variable.Is.RequestBufferReading) {
      // And it already took 1.5T from the last message
      if ((micros() - variable.Time.LastCommunication) >
        (variable.Time.HalfChar * MODBUS_HALF_SILENCE_MULTIPLIER)) {
        // start reading and clear buffer
        variable.Length.RequestBuffer = L();
        variable.Is.RequestBufferReading = true;
      } else {
        // discard data
        stream.read();
      }
    }

    // if already in reading
    if (variable.Is.RequestBufferReading) {
#ifdef MODBUS_MAX_BUFFER
      if (variable.Length.RequestBuffer == MODBUS_MAX_BUFFER) {
        // buffer is already full; stop reading
        variable.Is.RequestBufferReading = false;
      }
      // add new bytes to buffer
      length = min(length, MODBUS_MAX_BUFFER - variable.Length.RequestBuffer);
      length = (L)(stream.readBytes(
        request.Buffer + variable.Length.RequestBuffer,
        static_cast<size_t>(MODBUS_MAX_BUFFER - variable.Length.RequestBuffer)));
#else
      if (variable.Length.RequestBuffer >= request.Size) {
        // buffer is already full; stop reading
        variable.Is.RequestBufferReading = false;
      }
      // add new bytes to buffer
      length = min(length, request.Size - variable.Length.RequestBuffer);
      length = (L)(stream.readBytes(
        request.Buffer + variable.Length.RequestBuffer,
        static_cast<size_t>(request.Size - variable.Length.RequestBuffer)));
#endif

      // if this is the first read, check the address to reject irrelevant requests
      if (variable.Length.RequestBuffer == 0 &&
        length > MODBUS_ADDRESS_INDEX &&
        (request.Buffer[MODBUS_ADDRESS_INDEX] != parameter.SlaveId &&
          request.Buffer[MODBUS_ADDRESS_INDEX] != MODBUS_BROADCAST_ADDRESS)) {
        // bad address, stop reading
        variable.Is.RequestBufferReading = false;
      }

      // move byte pointer forward
      variable.Length.RequestBuffer += length;
      variable.Length.TotalReceived += length;
    }

    // save the time of last received byte(s)
    variable.Time.LastCommunication = micros();

    // wait for more data
    return false;
  } else {
    // if we are in reading but no data is available for 1.5T; this request is completed
    if (variable.Is.RequestBufferReading &&
      ((micros() - variable.Time.LastCommunication) >
      (variable.Time.HalfChar * MODBUS_HALF_SILENCE_MULTIPLIER))) {
      // allow for new requests to be processed
      variable.Is.RequestBufferReading = false;
    } else {
      // otherwise, wait
      return false;
    }
  }

  return variable.Is.RequestBufferReading &&
    (variable.Length.RequestBuffer >= MODBUS_FRAME_SIZE);
}
}

namespace report {
/**
 * Fills the output buffer with an exception in regard to the request already
 * in the input buffer and writes the response. No need to do it later.
 *
 * @param exceptionCode the status code to report.
 * @return the number of bytes written
 */
template<
  typename A = uint16_t,
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename P = int,
  typename R = uint8_t,
  typename S = uint16_t,
  typename T = uint64_t >
  R mexception(
    Stream& stream,
    const ::gos::atl::modbus::structures::Parameter<A, P> parameter,
    ::gos::atl::modbus::structures::Variable<I, L, M, T>& variable,
    ::gos::atl::buffer::Holder<S, char>& request,
    ::gos::atl::buffer::Holder<S, char>& response,
    const R& code) {
  // we don't respond to broadcast messages
  if (request.Buffer[MODBUS_ADDRESS_INDEX] == MODBUS_BROADCAST_ADDRESS)
  {
    return 0;
  }
  variable.Length.ResponseBuffer = MODBUS_FRAME_SIZE + 1;
  response.Buffer[MODBUS_FUNCTION_CODE_INDEX] |= 0x80;
  response.Buffer[MODBUS_DATA_INDEX] = code;

  return write::response(stream, parameter, variable, response);
}
}

namespace validate {
template<
  typename A = uint16_t,
  typename C = uint16_t,
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename P = int,
  typename R = uint8_t,
  typename S = uint16_t,
  typename T = uint64_t>
  bool request(
    Stream& stream,
    const ::gos::atl::modbus::structures::Parameter<A, P> parameter,
    ::gos::atl::modbus::structures::Variable<I, L, M, T>& variable,
    ::gos::atl::buffer::Holder<S, char>& request,
    ::gos::atl::buffer::Holder<S, char>& response) {
  // minimum buffer size (1 x Address, 1 x Function, n x Data, 2 x CRC)
  L expectedrequestbuffersize = MODBUS_FRAME_SIZE;
  // check data validity based on the function code
  switch (request.Buffer[MODBUS_FUNCTION_CODE_INDEX]) {
  case MODBUS_FC_READ_EXCEPTION_STATUS:
    // broadcast is not supported
    if (request.Buffer[MODBUS_ADDRESS_INDEX] == MODBUS_BROADCAST_ADDRESS) {
      // ignore
      return false;
    }
    break;
  case MODBUS_FC_READ_COILS:             // read coils (digital read)
  case MODBUS_FC_READ_DISCRETE_INPUT:    // read input state (digital read)
  case MODBUS_FC_READ_HOLDING_REGISTERS: // read holding registers (analog read)
  case MODBUS_FC_READ_INPUT_REGISTERS:   // read input registers (analog read)
      // broadcast is not supported
    if (request.Buffer[MODBUS_ADDRESS_INDEX] == MODBUS_BROADCAST_ADDRESS) {
      // ignore
      return false;
    }
    // (2 x Index, 2 x Count)
    expectedrequestbuffersize += 4;
    break;
  case MODBUS_FC_WRITE_COIL:     // write coils (digital write)
  case MODBUS_FC_WRITE_REGISTER: // write regosters (digital write)
      // (2 x Index, 2 x Count)
    expectedrequestbuffersize += 4;
    break;
  case MODBUS_FC_WRITE_MULTIPLE_COILS:
  case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
    // (2 x Index, 2 x Count, 1 x Bytes)
    expectedrequestbuffersize += 5;
    if (variable.Length.RequestBuffer >= expectedrequestbuffersize) {
      // (n x Bytes)
      expectedrequestbuffersize += request.Buffer[6];
    }
    break;
  default:
    // unknown command
    ::gos::atl::modbus::details::report::mexception<A, I, L, M, P, R, S, T>(
      stream,
      parameter,
      variable,
      request,
      response,
      MODBUS_STATUS_ILLEGAL_FUNCTION);
    return false;
  }

  if (variable.Length.RequestBuffer < expectedrequestbuffersize) {
    // data is smaller than expected, ignore
    return false;
  }

  // set correct data size
  variable.Length.RequestBuffer = expectedrequestbuffersize;

  // check crc
  C crc = MODBUS_READ_READCRC(request.Buffer, variable.Length.RequestBuffer);
  //C crc = ::gos::atl::utility::crc::read<C, L>(request, variable.Length.RequestBuffer);
  return ::gos::atl::utility::crc::calculate<C, I, S>(
    request,
    variable.Length.RequestBuffer - MODBUS_CRC_LENGTH) == crc;
}
}

namespace create {
template<
  typename A = uint16_t,
  typename F = uint16_t,
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename R = uint8_t,
  typename S = uint16_t,
  typename T = uint64_t >
  R response(
    ::gos::atl::modbus::Handler<A, F, L, R>& handler,
    ::gos::atl::modbus::structures::Variable<I, L, M, T>& variable,
    ::gos::atl::buffer::Holder<S, char>& request,
    ::gos::atl::buffer::Holder<S, char>& response) {
  A first;
  L length;
  I index;

  /**
  * Match the function code with a callback and execute it
  * as well as preparing the response buffer
  */
  switch (response.Buffer[MODBUS_FUNCTION_CODE_INDEX]) {
  case MODBUS_FC_READ_EXCEPTION_STATUS:
    // add response data length to output buffer length
    variable.Length.ResponseBuffer += 1;

    // execute callback and return the status code
#ifdef GOS_DEPRECATED
    return callback::execute<A, I, L, R>(MODBUS_CB_READ_EXCEPTION_STATUS, 0, 8);
#else
    return handler.ReadExceptionStatus(MODBUS_CB_READ_EXCEPTION_STATUS);
#endif
  case MODBUS_FC_READ_COILS:          // read coils (digital out state)
  case MODBUS_FC_READ_DISCRETE_INPUT: // read input state (digital in)
    // read the the first input address and the number of inputs
    first = (A)(MODBUS_READ_UINT16(request.Buffer, MODBUS_DATA_INDEX));
    /* first = read::uint16<A, I, S>(request, MODBUS_DATA_INDEX); */

    length = (L)(MODBUS_READ_UINT16(request.Buffer, MODBUS_DATA_INDEX + 2));
    /* length = read::uint16<L, I, S>(request, MODBUS_DATA_INDEX + 2); */

    // calculate response data length and add to output buffer length
    request.Buffer[MODBUS_DATA_INDEX] = (length / 8) + (length % 8 != 0);
    request.Buffer[MODBUS_DATA_INDEX] = (length / 8) + (length % 8 != 0);
    variable.Length.ResponseBuffer += 1 + request.Buffer[MODBUS_DATA_INDEX];

    // execute callback and return the status code
#ifdef GOS_DEPRECATED
    index = request.Buffer[MODBUS_FUNCTION_CODE_INDEX] ==
      MODBUS_FC_READ_COILS ? MODBUS_CB_READ_COILS : MODBUS_CB_READ_DISCRETE_INPUTS;
    return callback::execute<A, I, L, R>(index, first, length);
#else
    if (request.Buffer[MODBUS_FUNCTION_CODE_INDEX] ==
      MODBUS_FC_READ_COILS) {
      index = MODBUS_CB_READ_COILS;
      return handler.ReadCoils(index, first, length);
    } else {
      index = MODBUS_CB_READ_DISCRETE_INPUTS;
      return handler.ReadCoils(index, first, length);
    }
#endif
  case MODBUS_FC_READ_HOLDING_REGISTERS: // read holding registers (analog out state)
  case MODBUS_FC_READ_INPUT_REGISTERS:   // read input registers (analog in)
      // read the starting address and the number of inputs
    first = (A)(MODBUS_READ_UINT16(request.Buffer, MODBUS_DATA_INDEX));
    /* first = read::uint16<A, I, S>(request, MODBUS_DATA_INDEX); */
    length = (L)(MODBUS_READ_UINT16(request.Buffer, MODBUS_DATA_INDEX + 2));
    /* length = read::uint16<L, I, S>(request, MODBUS_DATA_INDEX + 2); */

    response.Buffer[MODBUS_DATA_INDEX] = 2 * length;

    // calculate response data length and add to output buffer length
    variable.Length.ResponseBuffer += 1 + response.Buffer[MODBUS_DATA_INDEX];

    // execute callback and return the status code
#ifdef GOS_DEPRECATED
    index = request.Buffer[MODBUS_FUNCTION_CODE_INDEX] ==
      MODBUS_FC_READ_HOLDING_REGISTERS ?
      MODBUS_CB_READ_HOLDING_REGISTERS :
      MODBUS_CB_READ_INPUT_REGISTERS;
    return callback::execute<A, I, L, R>(index, first, length);
#else
    if (request.Buffer[MODBUS_FUNCTION_CODE_INDEX] ==
      MODBUS_FC_READ_HOLDING_REGISTERS) {
      index = MODBUS_CB_READ_HOLDING_REGISTERS;
      return handler.ReadHoldingRegisters(index, first, length);
    } else {
      index = MODBUS_CB_READ_INPUT_REGISTERS;
      return handler.ReadInputRegisters(index, first, length);
    }
#endif
  case MODBUS_FC_WRITE_COIL: // write one coil (digital out)
      // read the address

    first = (A)(MODBUS_READ_UINT16(request.Buffer, MODBUS_DATA_INDEX));
    /* first = read::uint16<A, I, S>(request, MODBUS_DATA_INDEX); */

    // add response data length to output buffer length
    variable.Length.ResponseBuffer += 4;
    // copy parts of the request data that need to be in the response data
    ::memcpy(
      (void*)(response.Buffer + MODBUS_DATA_INDEX),
      (const void*)(request.Buffer + MODBUS_DATA_INDEX),
      variable.Length.ResponseBuffer - MODBUS_FRAME_SIZE);

    // execute callback and return the status code
#ifdef GOS_DEPRECATED
    return callback::execute<A, I, L, R>(CB_WRITE_COILS, first, 1);
#else
    return handler.WriteCoils(MODBUS_CB_WRITE_COILS, first, 1);
#endif
  case MODBUS_FC_WRITE_REGISTER:
    // read the address
    first = (A)(MODBUS_READ_UINT16(request.Buffer, MODBUS_DATA_INDEX));
    /* first = read::uint16<A, I, S>(request, MODBUS_DATA_INDEX); */

    // add response data length to output buffer length
    variable.Length.ResponseBuffer += 4;
    // copy parts of the request data that need to be in the response data
    ::memcpy(
      response.Buffer + MODBUS_DATA_INDEX,
      request.Buffer + MODBUS_DATA_INDEX,
      variable.Length.ResponseBuffer - MODBUS_FRAME_SIZE);

    // execute callback and return the status code
#ifdef GOS_DEPRECATED
    return callback::execute<A, I, L, R>(CB_WRITE_HOLDING_REGISTERS, first, 1);
#else
    return handler.WriteHoldingRegisters(MODBUS_CB_WRITE_HOLDING_REGISTERS, first, 1);
#endif
  case MODBUS_FC_WRITE_MULTIPLE_COILS: // write coils (digital out)
      // read the starting address and the number of outputs
    first = (A)(MODBUS_READ_UINT16(request.Buffer, MODBUS_DATA_INDEX));
    /* first = read::uint16<A, I, S>(request, MODBUS_DATA_INDEX); */
    length = (L)(MODBUS_READ_UINT16(request.Buffer, MODBUS_DATA_INDEX + 2));
    /* length = read::uint16<L, I, S>(request, MODBUS_DATA_INDEX + 2); */

    // add response data length to output buffer length
    variable.Length.ResponseBuffer += 4;
    // copy parts of the request data that need to be in the response data
    ::memcpy(
      response.Buffer + MODBUS_DATA_INDEX,
      request.Buffer + MODBUS_DATA_INDEX,
      variable.Length.ResponseBuffer - MODBUS_FRAME_SIZE);

    // execute callback and return the status code
#ifdef GOS_DEPRECATED
    return callback::execute<A, I, L, R>(CB_WRITE_COILS, first, length);
#else
    return handler.WriteCoils(MODBUS_CB_WRITE_COILS, first, length);
#endif
  case MODBUS_FC_WRITE_MULTIPLE_REGISTERS: // write holding registers (analog out)
      // read the starting address and the number of outputs
    first = (A)(MODBUS_READ_UINT16(request.Buffer, MODBUS_DATA_INDEX));
    /* first = read::uint16<A, I, S>(request, MODBUS_DATA_INDEX); */
    length = (L)(MODBUS_READ_UINT16(request.Buffer, MODBUS_DATA_INDEX + 2));
    /* length = read::uint16<L, I, S>(request, MODBUS_DATA_INDEX + 2); */

    // add response data length to output buffer length
    variable.Length.ResponseBuffer += 4;
    // copy parts of the request data that need to be in the response data
    ::memcpy(
      response.Buffer + MODBUS_DATA_INDEX,
      request.Buffer + MODBUS_DATA_INDEX,
      variable.Length.ResponseBuffer - MODBUS_FRAME_SIZE);

    // execute callback and return the status code
#ifdef GOS_DEPRECATED
    return callback::execute<A, I, L, R>(
      MODBUS_CB_WRITE_HOLDING_REGISTERS,
      first,
      length);
#else
    return handler.WriteHoldingRegisters(
      MODBUS_CB_WRITE_HOLDING_REGISTERS, first, length);
#endif
  default:
    return MODBUS_STATUS_ILLEGAL_FUNCTION;
  }

}
}

}

template<
  typename A = uint16_t,
  typename B = uint64_t,
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename P = int,
  typename R = uint8_t,
  typename T = uint64_t >
  void begin(
    Stream& stream,
    const ::gos::atl::modbus::structures::Parameter<A, P> parameter,
    ::gos::atl::modbus::structures::Variable<I, L, M, T>& variable,
    const B& rate) {
  variable.Index.ResponseWrite = I();
  variable.Length.TotalSent = L();
  variable.Length.TotalReceived = L();
  variable.Length.ResponseBuffer = L();
  variable.Is.RequestBufferReading = false;
  variable.Is.ResponseBufferWriting = false;
  pinMode(parameter.TransmissionControl, OUTPUT);
  digitalWrite(parameter.TransmissionControl, LOW);
  // disable serial stream timeout and cleans the buffer
  stream.setTimeout(0);
  stream.flush();
  variable.Length.TransmissionBuffer = stream.availableForWrite();
  // calculate half char time time from the serial's baudrate
  variable.Time.HalfChar = rate > 19200 ? 250 : 5000000 / rate; // 0.5T
  variable.Time.LastCommunication = micros() +
    (variable.Time.HalfChar * MODBUS_FULL_SILENCE_MULTIPLIER);
  variable.Length.RequestBuffer = L();
}

template<
  typename A = uint16_t,
  typename C = uint16_t,
  typename F = uint16_t,
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename P = int,
  typename R = uint8_t,
  typename S = uint16_t,
  typename T = uint64_t>
  R loop(
    Stream& stream,
    const ::gos::atl::modbus::structures::Parameter<A, P> parameter,
    ::gos::atl::modbus::Handler<A, F, L, R>& handler,
    ::gos::atl::modbus::structures::Variable<I, L, M, T>& variable,
    ::gos::atl::buffer::Holder<S, char>& request,
    ::gos::atl::buffer::Holder<S, char>& response) {
  if (variable.Is.ResponseBufferWriting) {
    ::gos::atl::modbus::details::write::response<A, C, I, L, M, P, S, T>(
      stream,
      parameter,
      variable,
      response);
  }

  if (!::gos::atl::modbus::details::read::request<A, I, L, M, P, S, T>(
    stream,
    parameter,
    variable,
    request)) {
    return R();
  }

  // prepare output buffer
  ::memset(response.Buffer, 0x00, response.Size);
  response.Buffer[MODBUS_ADDRESS_INDEX] =
    request.Buffer[MODBUS_ADDRESS_INDEX];
  response.Buffer[MODBUS_FUNCTION_CODE_INDEX] =
    request.Buffer[MODBUS_FUNCTION_CODE_INDEX];
  variable.Length.ResponseBuffer = MODBUS_FRAME_SIZE;

  // validate request
  if (!::gos::atl::modbus::details::validate::request<
    A, C, I, L, M, P, R, S, T>(
      stream,
      parameter,
      variable,
      request,
      response)) {
    return R();
  }

  // execute request and fill the response
  R status = ::gos::atl::modbus::details::create::response<
    A, F, I, L, M, R, S, T>(
      handler, variable, request, response);

  // check if the callback execution failed
  if (status != MODBUS_STATUS_OK) {
    return ::gos::atl::modbus::details::report::mexception<
      A, I, L, M, P, R, S, T>(
        stream, parameter, variable, request, response, status);
  }

  // writes the response being created
  return ::gos::atl::modbus::details::write::response<
    A, C, I, L, M, P, S, T>(stream, parameter, variable, response);
}

namespace index {
namespace access {
template<
  typename A = uint16_t,
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename R = uint8_t,
  typename S = uint16_t,
  typename T = uint64_t>
  R coil(
    ::gos::atl::modbus::structures::Variable<I, L, M, T>& variable,
    ::gos::atl::buffer::Holder<S, char>& request,
    const I& offset,
    I& index,
    I& bitindex) {
  if (request.Buffer[MODBUS_FUNCTION_CODE_INDEX] == MODBUS_FC_WRITE_COIL)
  {
    if (offset == 0)
    {
      index = MODBUS_DATA_INDEX + 2;
      // (2 x coilAddress, 1 x value)
      /* return MODBUS_READ_UINT16(request.Buffer, MODBUS_DATA_INDEX + 2) == MODBUS_COIL_ON; */
      return MODBUS_FC_WRITE_COIL;
    }
  } else if (request.Buffer[MODBUS_FUNCTION_CODE_INDEX] ==
    MODBUS_FC_WRITE_MULTIPLE_COILS) {
    // (2 x firstCoilAddress, 2 x coilsCount, 1 x valueBytes, n x values)
    index = MODBUS_DATA_INDEX + 5 + (offset / 8);
    bitindex = offset % 8;

    return index < variable.Length.RequestBuffer - MODBUS_CRC_LENGTH;
  }
}
template<
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename R = uint8_t,
  typename S = uint16_t,
  typename T = uint64_t>
  bool registers(
    ::gos::atl::modbus::structures::Variable<I, L, M, T>& variable,
    ::gos::atl::buffer::Holder<S, char>& request,
    const I& offset,
    I& index) {
  if (request.Buffer[MODBUS_FUNCTION_CODE_INDEX] == MODBUS_FC_WRITE_REGISTER) {
    if (offset == 0) {
      index = MODBUS_DATA_INDEX + 2;
      return true;
    }
  } else if (request.Buffer[MODBUS_FUNCTION_CODE_INDEX] ==
    MODBUS_FC_WRITE_MULTIPLE_REGISTERS) {
    // (2 x firstRegisterAddress, 2 x registersCount, 1 x valueBytes, n x values)
    index = MODBUS_DATA_INDEX + 5 + (offset * 2);

    // check offset
    if (index < variable.Length.RequestBuffer - MODBUS_CRC_LENGTH)
    {
      return true;
    }
  }
  return false;
}
}
namespace provide {
template<
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename R = uint8_t,
  typename S = uint16_t,
  typename T = uint64_t>
  R coil(
    ::gos::atl::modbus::structures::Variable<I, L, M, T>& variable,
    ::gos::atl::buffer::Holder<S, char>& request,
    ::gos::atl::buffer::Holder<S, char>& response,
    const I& offset,
    //  const bool& state,
    I& index,
    I& bitindex) {
  // check function code
  if (
    request.Buffer[MODBUS_FUNCTION_CODE_INDEX] != MODBUS_FC_READ_DISCRETE_INPUT &&
    request.Buffer[MODBUS_FUNCTION_CODE_INDEX] != MODBUS_FC_READ_COILS) {
    return MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  }

  // (1 x valueBytes, n x values)
  index = MODBUS_DATA_INDEX + 1 + (offset / 8);
  bitindex = offset % 8;

  // check offset
  if (index >= variable.Length.ResponseBuffer - MODBUS_CRC_LENGTH)
  {
    return MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  }

  return MODBUS_STATUS_OK;
}

template<
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename R = uint8_t,
  typename S = uint16_t,
  typename T = uint64_t>
  R registers(
    ::gos::atl::modbus::structures::Variable<I, L, M, T>& variable,
    ::gos::atl::buffer::Holder<S, char>& request,
    const I& offset,
    I& index) {
  // check function code
  if (request.Buffer[MODBUS_FUNCTION_CODE_INDEX] !=
    MODBUS_FC_READ_HOLDING_REGISTERS &&
    request.Buffer[MODBUS_FUNCTION_CODE_INDEX] !=
    MODBUS_FC_READ_INPUT_REGISTERS) {
    return MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  }
  // (1 x valueBytes, n x values)
  index = MODBUS_DATA_INDEX + 1 + (offset * 2);

  // check offset
  if ((index + 2) > (variable.Length.ResponseBuffer - MODBUS_CRC_LENGTH))
  {
    return MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  }

  return MODBUS_STATUS_OK;
}
}
}

namespace access {
template<
  typename F = uint16_t,
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename S = uint16_t,
  typename T = uint64_t>
  F function(
    ::gos::atl::modbus::structures::Variable<I, L, M, T>& variable,
    ::gos::atl::buffer::Holder<S, char>& request) {
  if (variable.Length.RequestBuffer >= MODBUS_FRAME_SIZE &&
    !variable.Is.RequestBufferReading)
  {
    return request.Buffer[MODBUS_FUNCTION_CODE_INDEX];
  }
  return MODBUS_FC_INVALID;
}

namespace unit {
template<
  typename A = uint16_t,
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename S = uint16_t,
  typename T = uint64_t>
  A address(
    ::gos::atl::modbus::structures::Variable<I, L, M, T>& variable,
    ::gos::atl::buffer::Holder<S, char>& request) {
  if (variable.Length.RequestBuffer >= MODBUS_FRAME_SIZE &&
    !variable.Is.RequestBufferReading)
  {
    return request.Buffer[MODBUS_ADDRESS_INDEX];
  }
  return MODBUS_FC_INVALID;
}
}

namespace broadcast {
template<
  typename A = uint16_t,
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename S = uint16_t,
  typename T = uint64_t>
  bool is(
    ::gos::atl::modbus::structures::Variable<I, L, M, T>& variable,
    ::gos::atl::buffer::Holder<S, char>& request) {
  return ::gos::atl::modbus::access::unit::address<A, I, L, M, S>(
    variable, request) == MODBUS_BROADCAST_ADDRESS;
}
}

template<
  typename A = uint16_t,
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename R = uint8_t,
  typename S = uint16_t,
  typename T = uint64_t>
  bool coil(
    ::gos::atl::modbus::structures::Variable<I, L, M, T>& variable,
    ::gos::atl::buffer::Holder<S, char>& request, const I& offset) {
  I index, bindex;
  if (::gos::atl::modbus::index::access::coil<A, I, L, M, R, S, T>(
    variable, request, offset, index, bindex) == MODBUS_STATUS_OK) {
    return bitRead(request.Buffer[index], bindex);
  } else {
    return false;
  }
}

template<
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename R = uint8_t,
  typename S = uint16_t,
  typename T = uint64_t,
  typename V = uint16_t>
  V registers(
    ::gos::atl::modbus::structures::Variable<I, L, M, T>& variable,
    ::gos::atl::buffer::Holder<S, char>& request,
    const I& offset) {
  I index;
  if (::gos::atl::modbus::index::access::registers<I, L, M, R, S, T>(
    variable, request, offset, index) == MODBUS_STATUS_OK) {
    return MODBUS_READ_UINT16(request.Buffer, index);
  } else {
    return 0;
  }
}
} // access namespace

namespace provide {

template<
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename R = uint8_t,
  typename S = uint16_t,
  typename T = uint64_t>
  R mexception(
    ::gos::atl::modbus::structures::Variable<I, L, M, T>& variable,
    ::gos::atl::buffer::Holder<S, char>& request,
    ::gos::atl::buffer::Holder<S, char>& response,
    const I& offset,
    const bool& status) {
  // check function code
  if (request.Buffer[MODBUS_FUNCTION_CODE_INDEX] !=
    MODBUS_FC_READ_EXCEPTION_STATUS) {
    return MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  }

  // (1 x values)
  I index = MODBUS_DATA_INDEX;
  I bitindex = offset % 8;

  // check offset
  if (index >= variable.Length.ResponseBuffer - MODBUS_CRC_LENGTH)
  {
    return MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  }

  if (status)
  {
    bitSet(response.Buffer[index], bitindex);
  } else
  {
    bitClear(response.Buffer[index], bitindex);
  }

  return MODBUS_STATUS_OK;
}

template<
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename R = uint8_t,
  typename S = uint16_t,
  typename T = uint64_t>
  R coil(
    ::gos::atl::modbus::structures::Variable<I, L, M, T>& variable,
    ::gos::atl::buffer::Holder<S, char>& request,
    ::gos::atl::buffer::Holder<S, char>& response,
    const I& offset,
    const bool& status) {
  I index, bitindex;
  R result = ::gos::atl::modbus::index::provide::coil<I, L, M, R, S, T>(
    variable, request, response, offset, index, bitindex);
  if (result == MODBUS_STATUS_OK) {
    if (status) {
      bitSet(response.Buffer[index], bitindex);
    } else {
      bitClear(response.Buffer[index], bitindex);
    }
  }
  return result;
}

template<
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename R = uint8_t,
  typename S = uint16_t,
  typename T = uint64_t>
  R discrete(
    ::gos::atl::modbus::structures::Variable<I, L, M, T>& variable,
    ::gos::atl::buffer::Holder<S, char>& request,
    ::gos::atl::buffer::Holder<S, char>& response,
    const I& offset,
    const bool& status) {
  return coil<I, L, M, R, S, T>(variable, request, response, offset, status);
}

template<
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename R = uint8_t,
  typename S = uint16_t,
  typename T = uint64_t,
  typename V = uint16_t>
  R registers(
    ::gos::atl::modbus::structures::Variable<I, L, M, T>& variable,
    ::gos::atl::buffer::Holder<S, char>& request,
    ::gos::atl::buffer::Holder<S, char>& response,
    const I& offset,
    const V& value) {
  // check function code
  if (request.Buffer[MODBUS_FUNCTION_CODE_INDEX] !=
    MODBUS_FC_READ_HOLDING_REGISTERS &&
    request.Buffer[MODBUS_FUNCTION_CODE_INDEX] !=
    MODBUS_FC_READ_INPUT_REGISTERS) {
    return MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  }

  // (1 x valueBytes, n x values)
  I index = MODBUS_DATA_INDEX + 1 + (offset * 2);

  // check offset
  if ((index + 2) > (variable.Length.ResponseBuffer - MODBUS_CRC_LENGTH))
  {
    return MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  }

  response.Buffer[index] = value >> 8;
  response.Buffer[index + 1] = value & 0xff;

  return MODBUS_STATUS_OK;
}

template<
  typename C = char,
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename R = uint8_t,
  typename S = uint16_t,
  typename T = uint64_t>
  R string(
    ::gos::atl::modbus::structures::Variable<I, L, M, T>& variable,
    ::gos::atl::buffer::Holder<S, char>& response,
    const I& offset,
    const C* string,
    const L& length) {
  // (1 x valueBytes, n x values)
  I index = MODBUS_DATA_INDEX + 1 + (offset * 2);

  // check string length.
  if ((index + length) > (variable.Length.ResponseBuffer - MODBUS_CRC_LENGTH))
  {
    return MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  }

  memcpy(response.Buffer + index, string, length);

  return MODBUS_STATUS_OK;
}


} // provide namespace

namespace convert {
template<typename R> R convert(const ::gos::atl::modbus::binding::result& result) {
  switch (result) {
  case ::gos::atl::modbus::binding::result::included:
    return MODBUS_STATUS_OK;
  default:
    return 0;
  }
}
}

namespace binding {
namespace coil {
template<
  typename B,
  typename A = uint16_t,
  typename C = uint8_t,
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename R = uint8_t,
  typename S = uint16_t,
  typename T = uint64_t>
  ::gos::atl::modbus::binding::result access(
    ::gos::atl::binding::reference<B, A, C>& binding,
    ::gos::atl::modbus::structures::Variable<I, L, M, T>& variable,
    ::gos::atl::buffer::Holder<S, char>& request,
    ::gos::atl::buffer::Holder<S, char>& response,
    const uint16_t& start,       // Start address for the modbus function
    const uint16_t& length) {    // Length of the buffer for the modbus function
  ::gos::atl::modbus::binding::result result;
  uint16_t address, first, last;
  uint8_t index;
  result = ::gos::atl::modbus::binding::detail::initializenew(
    binding, start, length, address, first, last, index);
  if (result == ::gos::atl::modbus::binding::result::included) {
    while (first < length && index < binding.count) {
      ::gos::atl::modbus::provide::coil<I, L, M, R, S, T>(
        variable, request, response, first, *(binding.pointers[index]));
      first++;
      index++;
    }
  }
  return result;
}
template<
  typename B,
  typename A = uint16_t,
  typename C = uint8_t,
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename R = uint8_t,
  typename S = uint16_t,
  typename T = uint64_t>
  ::gos::atl::modbus::binding::result assign(
    ::gos::atl::binding::reference<B, A, C>& binding,
    ::gos::atl::modbus::structures::Variable<I, L, M, T>& variable,
    ::gos::atl::buffer::Holder<S, char>& request,
    ::gos::atl::buffer::Holder<S, char>& response,
    const uint16_t& start,    // Start address for the modbus function
    const uint16_t& length,   // Length of the buffer for the modbus function
    uint16_t& address,
    uint16_t& first,
    uint16_t& last,
    uint8_t index) {
  ::gos::atl::modbus::binding::result result;
  result = ::gos::atl::modbus::binding::detail::initializenew(
    binding, start, length, address, first, last, index);
  if (result == ::gos::atl::modbus::binding::result::included) {
    while (first < length && index < binding.count) {
      *(binding.pointers[index]) =
        ::gos::atl::modbus::access::coil<A, I, L, M, R, S, T>(
          variable, request, first);
      first++;
      index++;
    }
  }
  return result;
}

} // coil namespace

namespace discrete {
template<
  typename B,
  typename A = uint16_t,
  typename C = uint8_t,
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename R = uint8_t,
  typename S = uint16_t,
  typename T = uint64_t>
  ::gos::atl::modbus::binding::result access(
    ::gos::atl::binding::reference<B, A, C>& binding,
    ::gos::atl::modbus::structures::Variable<I, L, M, T>& variable,
    ::gos::atl::buffer::Holder<S, char>& request,
    ::gos::atl::buffer::Holder<S, char>& response,
    const uint16_t& start,       // Start address for the modbus function
    const uint16_t& length) {    // Length of the buffer for the modbus function
  ::gos::atl::modbus::binding::result result;
  uint16_t address, first, last;
  uint8_t index;
  result = ::gos::atl::modbus::binding::detail::initializenew(
    binding, start, length, address, first, last, index);
  if (result == ::gos::atl::modbus::binding::result::included) {
    while (first < length && index < binding.count) {
      ::gos::atl::modbus::provide::discrete<I, L, M, R, S, T>(
        variable, request, response, first, *(binding.pointers[index]));
      first++;
      index++;
    }
  }
  return result;
}
}

namespace registers {
template<
  typename B,
  typename A = uint16_t,
  typename C = uint8_t,
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename R = uint8_t,
  typename S = uint16_t,
  typename T = uint64_t>
  ::gos::atl::modbus::binding::result access(
    ::gos::atl::binding::reference<B, A, C>& binding,
    ::gos::atl::modbus::structures::Variable<I, L, M, T>& variable,
    ::gos::atl::buffer::Holder<S, char>& request,
    ::gos::atl::buffer::Holder<S, char>& response,
    const uint16_t& start,       // Start address for the modbus function
    const uint16_t& length) {    // Length of the buffer for the modbus function
  ::gos::atl::modbus::binding::result result;
  uint16_t address, first, last;
  uint8_t index;
  result = ::gos::atl::modbus::binding::detail::initializenew(
    binding, start, length, address, first, last, index);
   if (result == ::gos::atl::modbus::binding::result::included) {
    while (first < length && index < binding.count) {
      ::gos::atl::modbus::provide::registers<I, L, M, R, S, T>(
        variable, request, response, first, *(binding.pointers[index]));
      first++;
      index++;
    }
  }
   return result;
}

namespace registers {
template<
  typename B,
  typename A = uint16_t,
  typename C = uint8_t,
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename R = uint8_t,
  typename S = uint16_t,
  typename T = uint64_t>
  ::gos::atl::modbus::binding::result access(
    ::gos::atl::binding::barray::reference<B, A, C>& binding,
    ::gos::atl::modbus::structures::Variable<I, L, M, T>& variable,
    ::gos::atl::buffer::Holder<S, char>& request,
    ::gos::atl::buffer::Holder<S, char>& response,
    const uint16_t& start,       // Start address for the modbus function
    const uint16_t& length) {    // Length of the buffer for the modbus function
  ::gos::atl::modbus::binding::result result;
  uint16_t address, first, last;
  uint8_t index;
  result = ::gos::atl::modbus::binding::detail::initializenew(
    binding, start, length, address, first, last, index);
  if (result == ::gos::atl::modbus::binding::result::included) {
    while (first < length && index < binding.count) {
      ::gos::atl::modbus::provide::registers<I, L, M, R, S, T>(
        variable, request, response, first, binding.pointers[index]);
      first++;
      index++;
    }
  }
  return result;
}

template<
  typename B,
  typename A = uint16_t,
  typename C = uint8_t,
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename R = uint8_t,
  typename S = uint16_t,
  typename T = uint64_t>
  ::gos::atl::modbus::binding::result assign(
    ::gos::atl::binding::reference<B, A, C>& binding,
    ::gos::atl::modbus::structures::Variable<I, L, M, T>& variable,
    ::gos::atl::buffer::Holder<S, char>& request,
    ::gos::atl::buffer::Holder<S, char>& response,
    const uint16_t& start,    // Start address for the modbus function
    const uint16_t& length,   // Length of the buffer for the modbus function
    uint16_t& address,
    uint16_t& first,
    uint16_t& last,
    uint8_t index) {
  ::gos::atl::modbus::binding::result result;
  result = ::gos::atl::modbus::binding::detail::initializenew(
    binding, start, length, address, first, last, index);
  if (result == ::gos::atl::modbus::binding::result::included) {
    while (first < length && index < binding.count) {
      *(binding.pointers[index]) =
        ::gos::atl::modbus::access::registers<A, I, L, M, R, S, T>(
          variable, request, first);
      first++;
      index++;
    }
  }
  return result;
}

template<
  typename B,
  typename A = uint16_t,
  typename C = uint8_t,
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename R = uint8_t,
  typename S = uint16_t,
  typename T = uint64_t>
  ::gos::atl::modbus::binding::result assign(
    ::gos::atl::binding::barray::reference<B, A, C>& binding,
    ::gos::atl::modbus::structures::Variable<I, L, M, T>& variable,
    ::gos::atl::buffer::Holder<S, char>& request,
    ::gos::atl::buffer::Holder<S, char>& response,
    const uint16_t& start,    // Start address for the modbus function
    const uint16_t& length,   // Length of the buffer for the modbus function
    uint16_t& address,
    uint16_t& first,
    uint16_t& last,
    uint8_t index) {
  ::gos::atl::modbus::binding::result result;
  result = ::gos::atl::modbus::binding::detail::initializenew(
    binding, start, length, address, first, last, index);
  if (result == ::gos::atl::modbus::binding::result::included) {
    while (first < length && index < binding.count) {
      binding.pointers[index] =
        ::gos::atl::modbus::access::registers<A, I, L, M, R, S, T>(
          variable, request, first);
      first++;
      index++;
    }
  }
  return result;
}


}

namespace two {
template<
  typename B,
  typename A = uint16_t,
  typename C = uint8_t,
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename R = uint8_t,
  typename S = uint16_t,
  typename T = uint64_t >
  ::gos::atl::modbus::binding::result access(
  ::gos::atl::binding::reference<B, A, C>& binding,
  ::gos::atl::modbus::structures::Variable<I, L, M, T>& variable,
  ::gos::atl::buffer::Holder<S, char>& request,
  ::gos::atl::buffer::Holder<S, char>& response,
  const uint16_t& start,       // Start address for the modbus function
  const uint16_t& length) {    // Length of the buffer for the modbus function
  ::gos::atl::modbus::binding::result result;
  uint16_t address, first, last;
  uint8_t index;
  result = ::gos::atl::modbus::binding::detail::initializenew(
    binding, start, length, address, first, last, index);
  if (result == ::gos::atl::modbus::binding::result::included) {
    while (first < length && index < binding.count) {
      ::gos::atl::modbus::provide::registers<I, L, M, R, S, T>(
        variable, request, response, first++,
        ::gos::atl::utility::number::part::first(*(binding.pointers[index])));
      ::gos::atl::modbus::provide::registers<I, L, M, R, S, T>(
        variable, request, response, first++,
        ::gos::atl::utility::number::part::second(*(binding.pointers[index])));
      index++;
    }
  }
  return result;
}

template<
  typename B,
  typename A = uint16_t,
  typename C = uint8_t,
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename R = uint8_t,
  typename S = uint16_t,
  typename T = uint64_t >
  ::gos::atl::modbus::binding::result access(
    ::gos::atl::binding::barray::reference<B, A, C>& binding,
    ::gos::atl::modbus::structures::Variable<I, L, M, T>& variable,
    ::gos::atl::buffer::Holder<S, char>& request,
    ::gos::atl::buffer::Holder<S, char>& response,
    const uint16_t& start,       // Start address for the modbus function
    const uint16_t& length) {    // Length of the buffer for the modbus function
  ::gos::atl::modbus::binding::result result;
  uint16_t address, first, last;
  uint8_t index;
  result = ::gos::atl::modbus::binding::detail::initializenew(
    binding, start, length, address, first, last, index);
  if (result == ::gos::atl::modbus::binding::result::included) {
    while (first < length && index < binding.count) {
      ::gos::atl::modbus::provide::registers<I, L, M, R, S, T>(
        variable, request, response, first++,
        ::gos::atl::utility::number::part::first(binding.pointers[index]));
      ::gos::atl::modbus::provide::registers<I, L, M, R, S, T>(
        variable, request, response, first++,
        ::gos::atl::utility::number::part::second(binding.pointers[index]));
      index++;
    }
  }
  return result;
}

template<
  typename B,
  typename A = uint16_t,
  typename C = uint8_t,
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename R = uint8_t,
  typename S = uint16_t,
  typename T = uint64_t>
  ::gos::atl::modbus::binding::result assign(
    ::gos::atl::binding::reference<B, A, C>& binding,
    ::gos::atl::modbus::structures::Variable<I, L, M, T>& variable,
    ::gos::atl::buffer::Holder<S, char>& request,
    ::gos::atl::buffer::Holder<S, char>& response,
    const uint16_t& start,    // Start address for the modbus function
    const uint16_t& length,   // Length of the buffer for the modbus function
    uint16_t& address,
    uint16_t& first,
    uint16_t& last,
    uint8_t index) {
  ::gos::atl::modbus::binding::result result;
  result = ::gos::atl::modbus::binding::detail::initializenew(
    binding, start, length, address, first, last, index);
  if (result == ::gos::atl::modbus::binding::result::included) {
    while (first < length && index < binding.count) {
      *(binding.pointers[index]) = ::gos::atl::utility::number::part::combine<uint16_t, T>(
        ::gos::atl::modbus::access::registers<I, L, M, R, S, T>(
          variable, request, first++),
        ::gos::atl::modbus::access::registers<I, L, M, R, S, T>(
          variable, request, first++));
      index++;
    }
  }
  return result;
}

template<
  typename B,
  typename A = uint16_t,
  typename C = uint8_t,
  typename I = uint16_t,
  typename L = uint16_t,
  typename M = uint16_t,
  typename R = uint8_t,
  typename S = uint16_t,
  typename T = uint64_t>
  ::gos::atl::modbus::binding::result assign(
    ::gos::atl::binding::reference<B, A, C>& binding,
    ::gos::atl::modbus::structures::Variable<I, L, M, T>& variable,
    ::gos::atl::buffer::Holder<S, char>& request,
    ::gos::atl::buffer::Holder<S, char>& response,
    const uint16_t& start,    // Start address for the modbus function
    const uint16_t& length,   // Length of the buffer for the modbus function
    uint16_t& address,
    uint16_t& first,
    uint16_t& last,
    uint8_t index) {
  ::gos::atl::modbus::binding::result result;
  result = ::gos::atl::modbus::binding::detail::initializenew(
    binding, start, length, address, first, last, index);
  if (result == ::gos::atl::modbus::binding::result::included) {
    while (first < length && index < binding.count) {
      binding.pointers[index] =
      ::gos::atl::utility::number::part::combine<uint16_t, T>(
        ::gos::atl::modbus::access::registers<I, L, M, R, S, T>(
          variable, request, first++),
        ::gos::atl::modbus::access::registers<I, L, M, R, S, T>(
          variable, request, first++));
      index++;
    }
  }
  return result;
}
}

#ifdef GOS_TODO_LATER
namespace discrete {
template<typename T> bool access(
  ::gos::atl::binding::reference<T, uint16_t, uint8_t>& binding,
  Modbus& modbus,
  const uint16_t& startaddress,
  const uint16_t& length) {
  uint8_t offset, from, to;
  bool re = detail::initialize(binding, startaddress, length, offset, from, to);
  while (from < to) {
    modbus.writeDiscreteInputToBuffer(offset++, *(binding.pointers[from++]));
  }
  return re;
}
} // descrete namespace

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
  uint8_t offset, index;
  bool re = detail::initialize(binding, startaddress, length, offset, from, to);
  index = from;
  while (index < to) {
    *(binding.pointers[index++]) = modbus.readRegisterFromBuffer(offset++);
  }
  return re;
}
template<typename T> bool assign(
  ::gos::atl::binding::change::aware::reference<T, uint16_t, uint8_t>& binding,
  Modbus& modbus,
  const uint16_t& startaddress,
  const uint16_t& length,
  uint8_t& from,
  uint8_t& to) {
  T value;
  uint8_t offset, index;
  bool result = detail::initialize(binding, startaddress, length, offset, from, to);
  index = from;
  while (index < to) {
    value = modbus.readRegisterFromBuffer(offset++);
    if (value != *(binding.pointers[index])) {
      *(binding.pointers[index]) = value;
      ::gos::atl::binding::change::aware::set<T, uint16_t, uint8_t>(
        binding, index, true);
    } else {
      ::gos::atl::binding::change::aware::set<T, uint16_t, uint8_t>(
        binding, index, false);
    }
    index++;
  }
  return result;
}
} // registers namespace

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
template<typename T> bool assign(
  ::gos::atl::binding::change::aware::reference<T, uint16_t, uint8_t>& binding,
  Modbus& modbus,
  const uint16_t& startaddress,
  const uint16_t& length,
  uint8_t& from,
  uint8_t& to) {
  uint8_t offset, index;
  bool result = detail::initialize(
    binding,
    startaddress,
    length,
    offset,
    from,
    to);
  index = from;
  T value;
  while (index < to && offset < length) {
    value = ::gos::atl::utility::number::part::combine<uint16_t, T>(
      modbus.readRegisterFromBuffer(offset),
      modbus.readRegisterFromBuffer(offset + 1)
      );
    if (value != *(binding.pointers[index])) {
      *(binding.pointers[index]) = value;
      ::gos::atl::binding::change::aware::set<T, uint16_t, uint8_t>(
        binding, index, true);
    } else {
      ::gos::atl::binding::change::aware::set<T, uint16_t, uint8_t>(
        binding, index, false);
    }
    offset += 2;
    index++;
  }
  to = index;
  return result;
}
} // two namespace

#endif
} // binding namespace

} // modbus namespace
}
}

#endif /* _GOS_ARDUINO_TEMPLATE_LIBRARY_MODBUS_H_ */
