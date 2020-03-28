#ifndef _GOS_ARDUINO_TEMPLATE_LIBRARY_MODBUS_H_
#define _GOS_ARDUINO_TEMPLATE_LIBRARY_MODBUS_H_

#include <Arduino.h>

#include <gatlbinding.h>
#include <gatlutility.h>
#include <gatlbuffer.h>

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

#define MODBUS_INVALID_UNIT_ADDRESS                            255
#define MODBUS_DEFAULT_UNIT_ADDRESS                              1

#define MODBUS_COIL_OFF                                     0x0000
#define MODBUS_COIL_ON                                      0xff00

/**
* Modbus function codes
*/
#define MODBUS_FC_INVALID                                        0
#define MODBUS_FC_READ_COILS                                     1
#define MODBUS_FC_READ_DISCRETE_INPUT                            2
#define MODBUS_FC_READ_HOLDING_REGISTERS                         3
#define MODBUS_FC_READ_INPUT_REGISTERS                           4
#define MODBUS_FC_WRITE_COIL                                     5
#define MODBUS_FC_WRITE_REGISTER                                 6
#define MODBUS_FC_READ_EXCEPTION_STATUS                          7
#define MODBUS_FC_WRITE_MULTIPLE_COILS                          15
#define MODBUS_FC_WRITE_MULTIPLE_REGISTERS                      16

/**
  * Modbus status codes
  */
#define MODBUS_STATUS_OK                                         0
#define MODBUS_STATUS_ILLEGAL_FUNCTION                           1
#define MODBUS_STATUS_ILLEGAL_DATA_ADDRESS                       2
#define MODBUS_STATUS_ILLEGAL_DATA_VALUE                         3
#define MODBUS_STATUS_SLAVE_DEVICE_FAILURE                       4
#define MODBUS_STATUS_ACKNOWLEDGE                                5
#define MODBUS_STATUS_SLAVE_DEVICE_BUSY                          6
#define MODBUS_STATUS_NEGATIVE_ACKNOWLEDGE                       7 
#define MODBUS_STATUS_MEMORY_PARITY_ERROR                        8
#define MODBUS_STATUS_GATEWAY_PATH_UNAVAILABLE                   9
#define MODBUS_STATUS_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND   10

#define MODBUS_FRAME_SIZE                                        4
#define MODBUS_CRC_LENGTH                                        2

#define MODBUS_ADDRESS_INDEX                                     0
#define MODBUS_FUNCTION_CODE_INDEX                               1
#define MODBUS_DATA_INDEX                                        2

#define MODBUS_BROADCAST_ADDRESS                                 0
#define MODBUS_ADDRESS_MIN                                       1
#define MODBUS_ADDRESS_MAX                                     247

#define MODBUS_HALF_SILENCE_MULTIPLIER                           3
#define MODBUS_FULL_SILENCE_MULTIPLIER                           7

#define MODBUS_READ_UINT16_AT0(arr) word(*arr, *(arr + 1))
#define MODBUS_READ_UINT16(arr, index) word(arr[index], arr[index + 1])
#define MODBUS_READ_READCRC(arr, length) word(arr[(length - MODBUS_CRC_LENGTH) \
  + 1], arr[length - MODBUS_CRC_LENGTH])

#define MODBUS_TYPE_DEFAULT                               uint16_t
#define MODBUS_TYPE_TIME                             unsigned long
#define MODBUS_TYPE_RATE                             unsigned long
#define MODBUS_TYPE_CODE                                   uint8_t
#define MODBUS_TYPE_RESULT                                 uint8_t
#define MODBUS_TYPE_FUNCTION                               uint8_t
#define MODBUS_TYPE_BIT_INDEX                              uint8_t
#define MODBUS_TYPE_BIND_INDEX                             uint8_t
#define MODBUS_TYPE_BUFFER                                 uint8_t
#define MODBUS_TYPE_PIN                                    uint8_t

namespace gos {
namespace atl {
namespace modbus {

namespace binding {
enum class result { undefined, included, excluded, failure };

template<typename T, typename S = uint8_t> S size() {
  return sizeof(T) / 2;
}

namespace detail {
namespace general {
template<typename T = MODBUS_TYPE_DEFAULT, typename I = MODBUS_TYPE_BIND_INDEX>
result initialize(
  const T& reference,   // First binding address
  const T& size,        // Binding size
  const T& count,       // Binding count
  const T& start,       // Start address for the modbus function
  const T& length,      // Length of the buffer for the modbus function
  T& address,           // The first modbus address that is affected
  T& first,             // The first buffer item to consider
  T& last,              // The last buffer item to consider
  I& index) {           // The first index in the binding to consider
  if (start > reference) {
    if (((start - reference) % size) == 0) {
      /* Using address to hold the difference temporarily */
      address = (start + length) - (reference + count * size);
      if (address > 0) {
        if ((address / size) < count) {
          last = length - address;
          address = start;
          first = 0;
          index = start - reference / size;
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
    address = (reference + count * size) - (start + length);
    if (address > 0) {
      if ((address % size) == 0) {
        address = reference;
        first = reference - start;
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
} // general namespace

template<
  typename B,
  typename T = MODBUS_TYPE_DEFAULT,
  typename I = MODBUS_TYPE_BIND_INDEX>
result initialize(
  const ::gos::atl::binding::reference<B, T, I>& reference,
  const T& start,       // Start address for the modbus function
  const T& length,      // Length of the buffer for the modbus function
  T& address,           // The first modbus address that is affected
  T& first,             // The first buffer item to consider
  T& last,              // The last buffer item to consider
  I& index) {           // The first index in the binding to consider
  return general::initialize<T>(
    reference.first,
    reference.size,
    reference.count,
    start,
    length,
    address,
    first,
    last,
    index);
}
template<
  typename B,
  typename T = MODBUS_TYPE_DEFAULT,
  typename I = MODBUS_TYPE_BIND_INDEX>
result initialize(
  const ::gos::atl::binding::barray::reference<B, T, I>& reference,
  const T& start,       // Start address for the modbus function
  const T& length,      // Length of the buffer for the modbus function
  T& address,           // The first modbus address that is affected
  T& first,             // The first buffer item to consider
  T& last,              // The last buffer item to consider
  I& index) {           // The first index in the binding to consider
  return general::initialize<T>(
    reference.first,
    reference.size,
    reference.count,
    start,
    length,
    address,
    first,
    last,
    index);
}

} // detail namespace
} // binding namespace

#ifdef MODBUS_HANDLER_INTERFACE
template<typename T = MODBUS_TYPE_DEFAULT > class Handler {
public:
  virtual ~Handler() {}
  virtual MODBUS_TYPE_RESULT ReadCoils(
    const T& address,
    const T& length) {
    return MODBUS_STATUS_ILLEGAL_FUNCTION;
  }
  virtual MODBUS_TYPE_RESULT ReadDiscreteInputs(
    const T& address,
    const T& length) {
    return MODBUS_STATUS_ILLEGAL_FUNCTION;
  }
  virtual MODBUS_TYPE_RESULT ReadHoldingRegisters(
    const T& address,
    const T& length) {
    return MODBUS_STATUS_ILLEGAL_FUNCTION;
  }
  virtual MODBUS_TYPE_RESULT ReadInputRegisters(
    const T& address,
    const T& length) {
    return MODBUS_STATUS_ILLEGAL_FUNCTION;
  }
  virtual MODBUS_TYPE_RESULT WriteCoils(
    const MODBUS_TYPE_FUNCTION& function,
    const T& address,
    const T& length) {
    return MODBUS_STATUS_ILLEGAL_FUNCTION;
  }
  virtual MODBUS_TYPE_RESULT WriteHoldingRegisters(
    const MODBUS_TYPE_FUNCTION& function,
    const T& address,
    const T& length) {
    return MODBUS_STATUS_ILLEGAL_FUNCTION;
  }
  virtual MODBUS_TYPE_RESULT ReadExceptionStatus() {
    return MODBUS_STATUS_ILLEGAL_FUNCTION;
  }
};
#else
namespace callback {
typedef MODBUS_TYPE_RESULT(*read)(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length);
typedef MODBUS_TYPE_RESULT(*write)(
  const MODBUS_TYPE_FUNCTION& function,
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length);
typedef MODBUS_TYPE_RESULT(*status)();
namespace detail {
namespace read {
static ::gos::atl::modbus::callback::read _coils = nullptr;
namespace discrete {
static ::gos::atl::modbus::callback::read _inputs = nullptr;
}
namespace holding {
static ::gos::atl::modbus::callback::read _registers = nullptr;
}
namespace input {
static ::gos::atl::modbus::callback::read _registers = nullptr;
}
namespace exception {
static ::gos::atl::modbus::callback::status _status = nullptr;
}
}
namespace write {
static ::gos::atl::modbus::callback::write _coils = nullptr;
namespace holding {
static ::gos::atl::modbus::callback::write _registers = nullptr;
}
}
}
namespace set {
namespace read {
template<typename T = MODBUS_TYPE_DEFAULT> void coils(
  const ::gos::atl::modbus::callback::read& function) {
  ::gos::atl::modbus::callback::detail::read::_coils = function;
}
namespace discrete {
template<typename T = MODBUS_TYPE_DEFAULT> void inputs(
  const ::gos::atl::modbus::callback::read& function) {
  ::gos::atl::modbus::callback::detail::read::discrete::_inputs = function;
}
}
namespace holding {
template<typename T = MODBUS_TYPE_DEFAULT> void registers(
  const ::gos::atl::modbus::callback::read& function) {
  ::gos::atl::modbus::callback::detail::read::holding::_registers = function;
}
}
namespace input {
template<typename T = MODBUS_TYPE_DEFAULT> void registers(
  const ::gos::atl::modbus::callback::read& function) {
  ::gos::atl::modbus::callback::detail::read::input::_registers = function;
}
}
namespace exception {
template<typename T = MODBUS_TYPE_DEFAULT> void status(
  const ::gos::atl::modbus::callback::status& function) {
  ::gos::atl::modbus::callback::detail::read::exception::_status = function;
}
}
}
namespace write {
template<typename T = MODBUS_TYPE_DEFAULT> void coils(
  const ::gos::atl::modbus::callback::write& function) {
  ::gos::atl::modbus::callback::detail::write::_coils = function;
}
namespace holding {
template<typename T = MODBUS_TYPE_DEFAULT> void registers(
  const ::gos::atl::modbus::callback::write& function) {
  ::gos::atl::modbus::callback::detail::write::holding::_registers = function;
}
}
}
}
}
#endif

namespace structures {
template<typename T = MODBUS_TYPE_DEFAULT> struct Parameter {
  T Id;
#ifndef MODBUS_NONE_CONTROL_PIN
  MODBUS_TYPE_PIN Control;
#endif
};

template<typename T = MODBUS_TYPE_DEFAULT> struct Index {
  T Write;
};

template<typename T = MODBUS_TYPE_DEFAULT> struct Length {
  typedef bool state;
  T Transmission;
  T Request;
  T Response;
};

template<typename T = MODBUS_TYPE_DEFAULT> struct Time {
  T Half;
  MODBUS_TYPE_TIME Last;
};

template< typename T = MODBUS_TYPE_DEFAULT> struct Variable {
  struct Time<T> Time;
  struct Length<T> Length;
  struct Index<T> Index;
  bool Writing;
  bool Reading;
};
} // structures namespace


namespace details {

namespace crc {
template<typename T = MODBUS_TYPE_DEFAULT> T calculate(
  ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& buffer, const T& length) {
  uint8_t j;
  T i;
  T crc = 0xFFFF;
  T tmp;
  // calculate crc
  for (i = 0; i < length; ++i) {
    crc ^= buffer.Buffer[i];
    for (j = 0; j < 8; ++j) {
      tmp = crc & 0x0001;
      crc = crc >> 1;
      if (tmp) {
        crc ^= 0xA001;
      }
    }
  }
  return crc;
}
} // namespace crc

namespace check {
namespace relevant {
template<typename T = MODBUS_TYPE_DEFAULT> inline bool address(
  const ::gos::atl::modbus::structures::Parameter<T> parameter,
  const MODBUS_TYPE_BUFFER& unit) {
  return unit == MODBUS_BROADCAST_ADDRESS || unit == parameter.Id;
}
}
}

namespace write {
/**
 * Writes the output buffer to serial stream
 *
 * @return The number of bytes written
 */
template<typename T = MODBUS_TYPE_DEFAULT> T response(
  Stream& stream,
  const ::gos::atl::modbus::structures::Parameter<T> parameter,
  ::gos::atl::modbus::structures::Variable<T>& variable,
  ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& response) {
  /**
  * Validate
  */
  // check if there is a response and this is supposed to be the first write
  if (variable.Index.Write == 0 &&
    variable.Length.Response >= MODBUS_FRAME_SIZE) {
    // set status as writing
    variable.Writing = true;
  }

  // check if we are not in writing or the address is broadcast
  if (!variable.Writing ||
    (response.Buffer[MODBUS_ADDRESS_INDEX]) == MODBUS_BROADCAST_ADDRESS) {
    // cleanup and ignore
    variable.Writing = false;
    variable.Index.Write = 0;
    variable.Length.Response = 0;
    return 0;
  }

  /**
    * Preparing
    */

    // if this is supposed to be the first write
  if (variable.Index.Write == 0) {
    // if we still need to wait
    if ((micros() - variable.Time.Last) <= (MODBUS_TYPE_TIME)
      (variable.Time.Half * MODBUS_HALF_SILENCE_MULTIPLIER)) {
      // ignore
      return 0;
    }

    // calculate and fill crc
    T crc = ::gos::atl::modbus::details::crc::calculate<T>(
      response,
      variable.Length.Response - MODBUS_CRC_LENGTH);
    response.Buffer[variable.Length.Response - MODBUS_CRC_LENGTH] =
      crc & 0xff;
    response.Buffer[(variable.Length.Response - MODBUS_CRC_LENGTH) + 1] = 
      crc >> 8;

    // enter transmission mode
#ifndef MODBUS_NONE_CONTROL_PIN
    digitalWrite(parameter.Control, HIGH);
#endif
  }

  /**
    * Transmit
    */

  // send buffer
  T length;
  if (variable.Length.Transmission > 0) {
    /* Her was a redeclaration of the a new length variable */
    length = min(
      stream.availableForWrite(),
      variable.Length.Response - variable.Index.Write);

    if (length > 0) {
      length = (T)(stream.write(
        (MODBUS_TYPE_BUFFER*)(response.Buffer + variable.Index.Write),
        static_cast<size_t>(length)));
      variable.Index.Write += length;
    }

    if (stream.availableForWrite() < variable.Length.Transmission)
    {
      // still waiting for write to complete
      variable.Time.Last = micros();
      return length;
    }

    // if buffer reports as empty; make sure it really is 
    // (`Serial` removes bytes from buffer before sending them)
    stream.flush();
    
    /* Set to zero to mimic the inner declaration of the original code */
    length = 0;
  } else {
    // compatibility for badly written software serials; aka AltSoftSerial
    length = variable.Length.Response - variable.Index.Write;

    if (length > 0) {
      length = (T)(stream.write(
        (const MODBUS_TYPE_BUFFER*)(response.Buffer),
        static_cast<size_t>(length)));
      stream.flush();
    }

    variable.Index.Write += length;
  }

  if (variable.Index.Write >= variable.Length.Response &&
    (micros() - variable.Time.Last) >
    (MODBUS_TYPE_TIME)(variable.Time.Half * MODBUS_HALF_SILENCE_MULTIPLIER)) {

    // end transmission
#ifndef MODBUS_NONE_CONTROL_PIN
    digitalWrite(parameter.Control, LOW);
#endif

    // cleanup
    variable.Writing = false;
    variable.Index.Write = 0;
    variable.Length.Response = 0;
  }

  return length;
}
} // write namespace

namespace read {
template<typename T = MODBUS_TYPE_DEFAULT> bool request(
  Stream& stream,
  const ::gos::atl::modbus::structures::Parameter<T> parameter,
  ::gos::atl::modbus::structures::Variable<T>& variable,
  ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& request) {
  /**
  * Read one data packet and report when received completely
  */
  T length = (T)(stream.available());

  if (length > 0) {
    // if not yet started reading
    if (!variable.Reading) {
      // And it already took 1.5T from the last message
      if ((micros() - variable.Time.Last) > (MODBUS_TYPE_TIME)
        (variable.Time.Half * MODBUS_HALF_SILENCE_MULTIPLIER)) {
        // start reading and clear buffer
        variable.Length.Request = 0;
        variable.Reading = true;
      } else {
        // discard data
        stream.read();
      }
    }

    // if already in reading
    if (variable.Reading) {
      if (variable.Length.Request >= request.Size) {
        // buffer is already full; stop reading
        variable.Reading = false;
      }
      // add new bytes to buffer
      length = min(length, request.Size - variable.Length.Request);
      /* Use to use the size - length and ignoring the min */
      length = (T)(stream.readBytes(
        request.Buffer + variable.Length.Request, length));

      // if this is the first read, check the address to reject irrelevant
      if (variable.Length.Request == 0 &&
        length > MODBUS_ADDRESS_INDEX &&
        !check::relevant::address(parameter, request.Buffer[MODBUS_ADDRESS_INDEX])) {
        // bad address, stop reading
        variable.Reading = false;
      }

      // move byte pointer forward
      variable.Length.Request += length;
    }

    // save the time of last received byte(s)
    variable.Time.Last = micros();

    // wait for more data
    return false;
  } else {
    // if we are in reading but no data is available for 1.5T
    if (variable.Reading && ((micros() - variable.Time.Last) >(MODBUS_TYPE_TIME)
      (variable.Time.Half * MODBUS_HALF_SILENCE_MULTIPLIER))) {
      // allow for new requests to be processed
      variable.Reading = false;
    } else {
      // otherwise, wait
      return false;
    }
  }

  return !variable.Reading && (variable.Length.Request >= MODBUS_FRAME_SIZE);
}
} // read namespace

namespace report {
/**
 * Fills the output buffer with an exception in regard to the request already
 * in the input buffer and writes the response. No need to do it later.
 *
 * @param exceptionCode the status code to report.
 * @return the number of bytes written
 */
template<typename T = MODBUS_TYPE_DEFAULT>
MODBUS_TYPE_RESULT mexception(
  Stream& stream,
  const ::gos::atl::modbus::structures::Parameter<T> parameter,
  ::gos::atl::modbus::structures::Variable<T>& variable,
  ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& request,
  ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& response,
  const MODBUS_TYPE_CODE& code) {
  // we don't respond to broadcast messages
  if (request.Buffer[MODBUS_ADDRESS_INDEX] == MODBUS_BROADCAST_ADDRESS) {
    return 0;
  }
  variable.Length.Response = MODBUS_FRAME_SIZE + 1;
  response.Buffer[MODBUS_FUNCTION_CODE_INDEX] |= 0x80;
  response.Buffer[MODBUS_DATA_INDEX] = code;

  return (MODBUS_TYPE_RESULT)write::response<T>(
    stream, parameter, variable, response);
}
} // report namespace

namespace validate {
template<typename T = MODBUS_TYPE_DEFAULT> bool request(
  Stream& stream,
  const ::gos::atl::modbus::structures::Parameter<T> parameter,
  ::gos::atl::modbus::structures::Variable<T>& variable,
  ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& request,
  ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& response) {
  // minimum buffer size (1 x Address, 1 x Function, n x Data, 2 x CRC)
  T expectedrequestbuffersize = MODBUS_FRAME_SIZE;
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
  case MODBUS_FC_WRITE_REGISTER: // write registers (digital write)
      // (2 x Index, 2 x Count)
    expectedrequestbuffersize += 4;
    break;
  case MODBUS_FC_WRITE_MULTIPLE_COILS:
  case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
    // (2 x Index, 2 x Count, 1 x Bytes)
    expectedrequestbuffersize += 5;
    if (variable.Length.Request >= expectedrequestbuffersize) {
      // (n x Bytes)
      expectedrequestbuffersize += request.Buffer[6];
    }
    break;
  default:
    // unknown command
    ::gos::atl::modbus::details::report::mexception<T>(
      stream,
      parameter,
      variable,
      request,
      response,
      MODBUS_STATUS_ILLEGAL_FUNCTION);
    return false;
  }

  if (variable.Length.Request < expectedrequestbuffersize) {
    // data is smaller than expected, ignore
    return false;
  }

  // set correct data size
  variable.Length.Request = expectedrequestbuffersize;

  // check crc
  T crc = MODBUS_READ_READCRC(request.Buffer, variable.Length.Request);
  return ::gos::atl::modbus::details::crc::calculate<T>(
    request, variable.Length.Request - MODBUS_CRC_LENGTH) == crc;
}
} // validate namespace

namespace create {
template<typename T = MODBUS_TYPE_DEFAULT> MODBUS_TYPE_RESULT response(
#ifdef MODBUS_HANDLER_INTERFACE
    ::gos::atl::modbus::Handler<T>& handler,
#endif
    ::gos::atl::modbus::structures::Variable<T>& variable,
    ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& request,
    ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& response) {
  T first, length;
  MODBUS_TYPE_BUFFER function = request.Buffer[MODBUS_FUNCTION_CODE_INDEX];

  /**
  * Match the function code with a callback and execute it
  * as well as preparing the response buffer
  */
  switch (function) {
  case MODBUS_FC_READ_EXCEPTION_STATUS:
    // add response data length to output buffer length
    variable.Length.Response += 1;

    // execute callback and return the status code
#ifdef MODBUS_HANDLER_INTERFACE
    return handler.ReadExceptionStatus();
#else
    if (::gos::atl::modbus::callback::detail::read::exception::_status) {
      return ::gos::atl::modbus::callback::detail::read::exception::_status();
    } else {
      return MODBUS_STATUS_ILLEGAL_FUNCTION;
    }
#endif
  case MODBUS_FC_READ_COILS:          // read coils (digital out state)
  case MODBUS_FC_READ_DISCRETE_INPUT: // read input state (digital in)
    // read the the first input address and the number of inputs
    first = MODBUS_READ_UINT16(request.Buffer, MODBUS_DATA_INDEX);
    length = MODBUS_READ_UINT16(request.Buffer, MODBUS_DATA_INDEX + 2);

    // calculate response data length and add to output buffer length
    response.Buffer[MODBUS_DATA_INDEX] = (length / 8) + (length % 8 != 0);
    variable.Length.Response += 1 + (response.Buffer[MODBUS_DATA_INDEX]);

    // execute callback and return the status code
#ifdef MODBUS_HANDLER_INTERFACE
    if (function == MODBUS_FC_READ_COILS) {
      return handler.ReadCoils(first, length);
    } else {
      return handler.ReadDiscreteInputs(first, length);
    }
#else
    if (function == MODBUS_FC_READ_COILS) {
      if (::gos::atl::modbus::callback::detail::read::_coils) {
        return ::gos::atl::modbus::callback::detail::read::_coils(
          first,
          length);
      } else {
        return MODBUS_STATUS_ILLEGAL_FUNCTION;
      }
    } else {
      if (::gos::atl::modbus::callback::detail::read::discrete::_inputs) {
        return ::gos::atl::modbus::callback::detail::read::discrete::_inputs(
          first,
          length);
      } else {
        return MODBUS_STATUS_ILLEGAL_FUNCTION;
      }
    }
#endif
  case MODBUS_FC_READ_HOLDING_REGISTERS: // read holding registers
  case MODBUS_FC_READ_INPUT_REGISTERS:   // read input registers
    // read the starting address and the number of inputs
    first = MODBUS_READ_UINT16(request.Buffer, MODBUS_DATA_INDEX);
    length = MODBUS_READ_UINT16(request.Buffer, MODBUS_DATA_INDEX + 2);

    // calculate response data length and add to output buffer length
    response.Buffer[MODBUS_DATA_INDEX] = 2 * length;
    variable.Length.Response += 1 + (response.Buffer[MODBUS_DATA_INDEX]);

    // execute callback and return the status code
#ifdef MODBUS_HANDLER_INTERFACE
    if (function == MODBUS_FC_READ_HOLDING_REGISTERS) {
      return handler.ReadHoldingRegisters(first, length);
    } else {
      return handler.ReadInputRegisters(first, length);
    }
#else
    if (function == MODBUS_FC_READ_HOLDING_REGISTERS) {
      if (::gos::atl::modbus::callback::detail::read::holding::_registers) {
        return ::gos::atl::modbus::callback::detail::read::holding::_registers(
          first,
          length);
      } else {
        return MODBUS_STATUS_ILLEGAL_FUNCTION;
      }
    } else {
      if (::gos::atl::modbus::callback::detail::read::input::_registers) {
        return ::gos::atl::modbus::callback::detail::read::input::_registers(
          first,
          length);
      } else {
        return MODBUS_STATUS_ILLEGAL_FUNCTION;
      }
    }

#endif
  case MODBUS_FC_WRITE_COIL: // write one coil (digital out)
    // read the address
    first = MODBUS_READ_UINT16(request.Buffer, MODBUS_DATA_INDEX);

    // add response data length to output buffer length
    variable.Length.Response += 4;
    // copy parts of the request data that need to be in the response data
    ::memcpy(
      (void*)((response.Buffer) + MODBUS_DATA_INDEX),
      (const void*)((request.Buffer) + MODBUS_DATA_INDEX),
      variable.Length.Response - MODBUS_FRAME_SIZE);

    // execute callback and return the status code
#ifdef MODBUS_HANDLER_INTERFACE
    return handler.WriteCoils(MODBUS_FC_WRITE_COIL, first, 1);
#else
    if (::gos::atl::modbus::callback::detail::write::_coils) {
      return ::gos::atl::modbus::callback::detail::write::_coils(
        MODBUS_FC_WRITE_COIL,
        first,
        1);
    } else {
      return MODBUS_STATUS_ILLEGAL_FUNCTION;
    }
#endif
  case MODBUS_FC_WRITE_REGISTER:
    // read the address
    first = MODBUS_READ_UINT16(request.Buffer, MODBUS_DATA_INDEX);

    // add response data length to output buffer length
    variable.Length.Response += 4;
    // copy parts of the request data that need to be in the response data
    ::memcpy(
      (void*)((response.Buffer) + MODBUS_DATA_INDEX),
      (const void*)((request.Buffer) + MODBUS_DATA_INDEX),
      variable.Length.Response - MODBUS_FRAME_SIZE);

    // execute callback and return the status code
#ifdef MODBUS_HANDLER_INTERFACE
    return handler.WriteHoldingRegisters(MODBUS_FC_WRITE_REGISTER, first, 1);
#else
    if (::gos::atl::modbus::callback::detail::write::holding::_registers) {
      return ::gos::atl::modbus::callback::detail::write::holding::_registers(
        MODBUS_FC_WRITE_REGISTER,
        first,
        1);
    } else {
      return MODBUS_STATUS_ILLEGAL_FUNCTION;
    }
#endif
  case MODBUS_FC_WRITE_MULTIPLE_COILS: // write coils (digital out)
    // read the starting address and the number of outputs
    first = MODBUS_READ_UINT16(request.Buffer, MODBUS_DATA_INDEX);
    length = MODBUS_READ_UINT16(request.Buffer, MODBUS_DATA_INDEX + 2);

    // add response data length to output buffer length
    variable.Length.Response += 4;
    // copy parts of the request data that need to be in the response data
    ::memcpy(
      (void*)((response.Buffer) + MODBUS_DATA_INDEX),
      (const void*)((request.Buffer) + MODBUS_DATA_INDEX),
      variable.Length.Response - MODBUS_FRAME_SIZE);

    // execute callback and return the status code
#ifdef MODBUS_HANDLER_INTERFACE
    return handler.WriteCoils(MODBUS_FC_WRITE_MULTIPLE_COILS, first, length);
#else
    if (::gos::atl::modbus::callback::detail::write::_coils) {
      return ::gos::atl::modbus::callback::detail::write::_coils(
        MODBUS_FC_WRITE_COIL,
        first,
        length);
    } else {
      return MODBUS_STATUS_ILLEGAL_FUNCTION;
    }
#endif
  case MODBUS_FC_WRITE_MULTIPLE_REGISTERS: // write holding registers
    // read the starting address and the number of outputs
    first = MODBUS_READ_UINT16(request.Buffer, MODBUS_DATA_INDEX);
    length = MODBUS_READ_UINT16(request.Buffer, MODBUS_DATA_INDEX + 2);

    // add response data length to output buffer length
    variable.Length.Response += 4;
    // copy parts of the request data that need to be in the response data
    ::memcpy(
      (void*)((response.Buffer) + MODBUS_DATA_INDEX),
      (const void*)((request.Buffer) + MODBUS_DATA_INDEX),
      variable.Length.Response - MODBUS_FRAME_SIZE);

    // execute callback and return the status code
#ifdef MODBUS_HANDLER_INTERFACE
    return handler.WriteHoldingRegisters(
      MODBUS_FC_WRITE_MULTIPLE_REGISTERS, first, length);
#else
    if (::gos::atl::modbus::callback::detail::write::holding::_registers) {
      return ::gos::atl::modbus::callback::detail::write::holding::_registers(
        MODBUS_FC_WRITE_REGISTER,
        first,
        length);
    } else {
      return MODBUS_STATUS_ILLEGAL_FUNCTION;
    }
#endif
  default:
    return MODBUS_STATUS_ILLEGAL_FUNCTION;
  }
}
} // create namespace

} // details namespace


template<typename T = MODBUS_TYPE_DEFAULT> void begin(
  Stream& stream,
  const ::gos::atl::modbus::structures::Parameter<T> parameter,
  ::gos::atl::modbus::structures::Variable<T>& variable,
  const MODBUS_TYPE_RATE& rate) {
  variable.Index.Write = 0;
  variable.Length.Request = 0;
  variable.Length.Response = 0;
  variable.Reading = false;
  variable.Writing = false;
#ifndef MODBUS_NONE_CONTROL_PIN
  pinMode(parameter.Control, OUTPUT);
  digitalWrite(parameter.Control, LOW);
#endif
  // disable serial stream timeout and cleans the buffer
  stream.setTimeout(0);
  stream.flush();
  variable.Length.Transmission = stream.availableForWrite();
  // calculate half char time from the serial's baud-rate
  variable.Time.Half = rate > 19200 ? 250 :
    static_cast<T>(5000000 / rate); // 0.5T
  variable.Time.Last = micros() +
    (variable.Time.Half * MODBUS_FULL_SILENCE_MULTIPLIER);
  /* The request buffer length has already been set to zero above */
}

template<typename T = MODBUS_TYPE_DEFAULT> T loop(
    Stream& stream,
    const ::gos::atl::modbus::structures::Parameter<T> parameter,
#ifdef MODBUS_HANDLER_INTERFACE
    ::gos::atl::modbus::Handler<T>& handler,
#endif
    ::gos::atl::modbus::structures::Variable<T>& variable,
    ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& request,
    ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& response) {

  // If we are in the writing, let it end first
  if (variable.Writing) {
    return ::gos::atl::modbus::details::write::response<T>(
      stream,
      parameter,
      variable,
      response);
  }

  if (!::gos::atl::modbus::details::read::request<T>(
    stream,
    parameter,
    variable,
    request)) {
    return 0;
  }

  // prepare output buffer
  ::gos::atl::buffer::clear(response);
  response.Buffer[MODBUS_ADDRESS_INDEX] =
    request.Buffer[MODBUS_ADDRESS_INDEX];
  response.Buffer[MODBUS_FUNCTION_CODE_INDEX] =
    request.Buffer[MODBUS_FUNCTION_CODE_INDEX];
  variable.Length.Response = MODBUS_FRAME_SIZE;

  // validate request
  if (!::gos::atl::modbus::details::validate::request<T>(
      stream,
      parameter,
      variable,
      request,
      response)) {
    return 0;
  }

  // execute request and fill the response
#ifdef MODBUS_HANDLER_INTERFACE
  MODBUS_TYPE_RESULT status = ::gos::atl::modbus::details::create::response<T>(
      handler, variable, request, response);
#else
  MODBUS_TYPE_RESULT status = ::gos::atl::modbus::details::create::response<T>(
    variable, request, response);
#endif

  // check if the callback execution failed
  if (status != MODBUS_STATUS_OK) {
    return ::gos::atl::modbus::details::report::mexception<T>(
      stream,
      parameter,
      variable,
      request,
      response,
      status);
  }

  // writes the response being created
  return ::gos::atl::modbus::details::write::response<T>(
    stream, parameter, variable, response);
}


namespace index {
namespace access {
/* readRegisterFromBuffer probably has bug for Write (single) Register */
template<typename T = MODBUS_TYPE_DEFAULT> MODBUS_TYPE_RESULT registers(
  ::gos::atl::modbus::structures::Variable<T>& variable,
  ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& request,
  const T& offset) {
  if (request.Buffer[MODBUS_FUNCTION_CODE_INDEX] == MODBUS_FC_WRITE_REGISTER) {
    if (offset == 0) {
      // (2 x coilAddress, 2 x value)
      return MODBUS_DATA_INDEX + 2;
    }
  } else if (request.Buffer[MODBUS_FUNCTION_CODE_INDEX] ==
    MODBUS_FC_WRITE_MULTIPLE_REGISTERS) {
    // (2 x firstRegisterAddress, 2 x registersCount, 1 x valueBytes, n x values)
    T index = MODBUS_DATA_INDEX + 5 + (offset * 2);
    // check offset
    if (index < variable.Length.Request - MODBUS_CRC_LENGTH) {
      return index;
    }
  }
  return 0;
}
}
namespace provide {
template<typename T = MODBUS_TYPE_DEFAULT> MODBUS_TYPE_RESULT digital(
    ::gos::atl::modbus::structures::Variable<T>& variable,
    ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& request,
    ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& response,
    const T& offset,
    //  const bool& state,
    T& index,
    MODBUS_TYPE_BIT_INDEX& bitindex) {
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
  if (index >= variable.Length.Response - MODBUS_CRC_LENGTH) {
    return MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  }

  return MODBUS_STATUS_OK;
}

template<typename T = MODBUS_TYPE_DEFAULT> MODBUS_TYPE_RESULT registers(
    ::gos::atl::modbus::structures::Variable<T>& variable,
    ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& request,
    const T& offset,
    T& index) {
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
  if ((index + 2) > (variable.Length.Response - MODBUS_CRC_LENGTH)) {
    return MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  }

  return MODBUS_STATUS_OK;
}
} // provide namespace
} // index namespace


namespace access {
template<typename T = MODBUS_TYPE_DEFAULT> MODBUS_TYPE_FUNCTION function(
  ::gos::atl::modbus::structures::Variable<T>& variable,
  ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& request) {
  if (variable.Length.Request >= MODBUS_FRAME_SIZE && !variable.Reading) {
    return request.Buffer[MODBUS_FUNCTION_CODE_INDEX];
  }
  return MODBUS_FC_INVALID;
}

namespace unit {
template<typename T = MODBUS_TYPE_DEFAULT> MODBUS_TYPE_RESULT address(
  ::gos::atl::modbus::structures::Variable<T>& variable,
  ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& request) {
  if (variable.Length.Request  >= MODBUS_FRAME_SIZE && !variable.Reading) {
    return request.Buffer[MODBUS_ADDRESS_INDEX];
  }
  return MODBUS_INVALID_UNIT_ADDRESS;
}
}

namespace buffer {
template<typename T = MODBUS_TYPE_DEFAULT> MODBUS_TYPE_BUFFER* location(
  ::gos::atl::modbus::structures::Variable<T>& variable,
  ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& request,
  MODBUS_TYPE_DEFAULT location,
  const MODBUS_TYPE_DEFAULT& address) {
  if ((location -= address) >= 0) {
    T index = ::gos::atl::modbus::index::access::registers(
      variable, request, location);
    return request.Buffer + index;
  }
  return nullptr;
}

template<typename T = MODBUS_TYPE_DEFAULT> MODBUS_TYPE_BUFFER* location(
  ::gos::atl::modbus::structures::Variable<T>& variable,
  ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& request,
  MODBUS_TYPE_DEFAULT location,
  const MODBUS_TYPE_DEFAULT& size,
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  if (location - address >= 0 && location + size <= address + length) {
    T index = ::gos::atl::modbus::index::access::registers(
      variable, request, location - address);
  }
  return nullptr;
}
}

namespace broadcast {
template<typename T = MODBUS_TYPE_DEFAULT> bool is(
  ::gos::atl::modbus::structures::Variable<T>& variable,
  ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& request) {
  return ::gos::atl::modbus::access::unit::address<T>(variable, request) ==
    MODBUS_BROADCAST_ADDRESS;
}
}

/* readCoilFromBuffer probably has bug for Write (single) Coil */
template<typename T = MODBUS_TYPE_DEFAULT> bool coil(
  ::gos::atl::modbus::structures::Variable<T>& variable,
  ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& request,
  const T& offset) {
  if (request.Buffer[MODBUS_FUNCTION_CODE_INDEX] == MODBUS_FC_WRITE_COIL) {
    if (offset == 0) {
      // (2 x coilAddress, 1 x value)
      return MODBUS_READ_UINT16(request.Buffer, MODBUS_DATA_INDEX + 2) ==
        MODBUS_COIL_ON;
    } else {
      return false;
    }
  } else if (request.Buffer[MODBUS_FUNCTION_CODE_INDEX] ==
    MODBUS_FC_WRITE_MULTIPLE_COILS) {
    // (2 x firstCoilAddress, 2 x coilsCount, 1 x valueBytes, n x values)
    T index = MODBUS_DATA_INDEX + 5 + (offset / 8);
    MODBUS_TYPE_BIT_INDEX bitindex = offset % 8;
    // check offset
    if (index < variable.Length.Request - MODBUS_CRC_LENGTH) {
      return bitRead(request.Buffer[index], bitindex);
    }
  }
  return false;
}

/* readRegisterFromBuffer probably has bug for Write (single) Register */
template<typename T = MODBUS_TYPE_DEFAULT> T registers(
    ::gos::atl::modbus::structures::Variable<T>& variable,
    ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& request,
    const T& offset) {
  T index = ::gos::atl::modbus::index::access::registers(
    variable, request, offset);
  if (index) {
    return MODBUS_READ_UINT16(request.Buffer, index);
  } else {
    return 0;
  }
}
} // access namespace

namespace provide {

template<typename T = MODBUS_TYPE_DEFAULT> MODBUS_TYPE_RESULT mexception(
    ::gos::atl::modbus::structures::Variable<T>& variable,
    ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& request,
    ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& response,
    const T& offset,
    const bool& status) {
  // check function code
  if (request.Buffer[MODBUS_FUNCTION_CODE_INDEX] !=
    MODBUS_FC_READ_EXCEPTION_STATUS) {
    return MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  }

  // (1 x values)
  T index = MODBUS_DATA_INDEX;
  T bitindex = offset % 8;

  // check offset
  if (index >= variable.Length.Response - MODBUS_CRC_LENGTH) {
    return MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  }

  if (status) {
    bitSet((response.Buffer[index]), bitindex);
  } else {
    bitClear((response.Buffer[index]), bitindex);
  }

  return MODBUS_STATUS_OK;
}

template<typename T = MODBUS_TYPE_DEFAULT> MODBUS_TYPE_RESULT coil(
    ::gos::atl::modbus::structures::Variable<T>& variable,
    ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& request,
    ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& response,
    const T& offset,
    const bool& status) {
  T index;
  MODBUS_TYPE_BIT_INDEX bitindex;
  MODBUS_TYPE_RESULT result = ::gos::atl::modbus::index::provide::digital<T>(
    variable, request, response, offset, index, bitindex);
  if (result == MODBUS_STATUS_OK) {
    if (status) {
      bitSet((response.Buffer[index]), bitindex);
    } else {
      bitClear((response.Buffer[index]), bitindex);
    }
  }
  return result;
}

template<typename T = MODBUS_TYPE_DEFAULT> MODBUS_TYPE_RESULT discrete(
    ::gos::atl::modbus::structures::Variable<T>& variable,
    ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& request,
    ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& response,
    const T& offset,
    const bool& status) {
  return coil<T>(variable, request, response, offset, status);
}

template<typename T = MODBUS_TYPE_DEFAULT> MODBUS_TYPE_RESULT registers(
    ::gos::atl::modbus::structures::Variable<T>& variable,
    ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& request,
    ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& response,
    const T& offset,
    const T& value) {
  T index;
  MODBUS_TYPE_RESULT result = ::gos::atl::modbus::index::provide::registers<T>(
    variable, request, offset, index);

  if (result == MODBUS_STATUS_OK) {
    response.Buffer[index] = value >> 8;
    response.Buffer[index + 1] = value & 0xff;
  }

  return result;
}

template<typename T = MODBUS_TYPE_DEFAULT> MODBUS_TYPE_RESULT string(
    ::gos::atl::modbus::structures::Variable<T>& variable,
    ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& response,
    const T& offset,
    const char* string,
    const T& length) {
  // (1 x valueBytes, n x values)
  T index = MODBUS_DATA_INDEX + 1 + (offset * 2);

  // check string length.
  if ((index + length) > (variable.Length.Response - MODBUS_CRC_LENGTH)) {
    return MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  }

  memcpy(response.Buffer + index, string, length);

  return MODBUS_STATUS_OK;
}

} // provide namespace


namespace convert {
template<typename R = MODBUS_TYPE_RESULT>
R convert(const ::gos::atl::modbus::binding::result& result) {
  switch (result) {
  case ::gos::atl::modbus::binding::result::included:
    return MODBUS_STATUS_OK;
  default:
    return 0;
  }
}
} // convert namespace


namespace binding {
namespace coil {
template<typename B, typename T = MODBUS_TYPE_DEFAULT>
::gos::atl::modbus::binding::result access(
  ::gos::atl::binding::reference<B, T, MODBUS_TYPE_BIND_INDEX>& binding,
  ::gos::atl::modbus::structures::Variable<T>& variable,
  ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& request,
  ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& response,
  const T& start,       // Start address for the modbus function
  const T& length) {    // Length of the buffer for the modbus function
  ::gos::atl::modbus::binding::result result;
  MODBUS_TYPE_DEFAULT address, first, last;
  MODBUS_TYPE_BIND_INDEX index;
  result = ::gos::atl::modbus::binding::detail::initialize<
    B, T, MODBUS_TYPE_BIND_INDEX>(
    binding, start, length, address, first, last, index);
  if (result == ::gos::atl::modbus::binding::result::included) {
    while (first < length && index < binding.count) {
      ::gos::atl::modbus::provide::coil<T>(
        variable, request, response, first, *(binding.pointers[index]));
      first++;
      index++;
    }
  }
  return result;
}
template<typename B, typename T = MODBUS_TYPE_DEFAULT>
::gos::atl::modbus::binding::result assign(
  ::gos::atl::binding::reference<B, T, MODBUS_TYPE_BIND_INDEX>& binding,
  ::gos::atl::modbus::structures::Variable<T>& variable,
  ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& request,
  ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& response,
  const T& start,    // Start address for the modbus function
  const T& length,   // Length of the buffer for the modbus function
  MODBUS_TYPE_DEFAULT& address,
  MODBUS_TYPE_DEFAULT& first,
  MODBUS_TYPE_DEFAULT& last,
  MODBUS_TYPE_BIND_INDEX index) {
  ::gos::atl::modbus::binding::result result;
  result = ::gos::atl::modbus::binding::detail::initialize<
    B, T, MODBUS_TYPE_BIND_INDEX>(
    binding, start, length, address, first, last, index);
  if (result == ::gos::atl::modbus::binding::result::included) {
    while (first < length && index < binding.count) {
      *(binding.pointers[index]) =
        ::gos::atl::modbus::access::coil<T>(variable, request, first);
      first++;
      index++;
    }
  }
  return result;
}

} // coil namespace

namespace discrete {
template<typename B, typename T = MODBUS_TYPE_DEFAULT>
::gos::atl::modbus::binding::result access(
  ::gos::atl::binding::reference<B, T, MODBUS_TYPE_BIND_INDEX>& binding,
  ::gos::atl::modbus::structures::Variable<T>& variable,
  ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& request,
  ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& response,
  const T& start,       // Start address for the modbus function
  const T& length) {    // Length of the buffer for the modbus function
  ::gos::atl::modbus::binding::result result;
  uint16_t address, first, last;
  MODBUS_TYPE_BIND_INDEX index;
  result = ::gos::atl::modbus::binding::detail::initialize<
    B, T, MODBUS_TYPE_BIND_INDEX>(
    binding, start, length, address, first, last, index);
  if (result == ::gos::atl::modbus::binding::result::included) {
    while (first < length && index < binding.count) {
      ::gos::atl::modbus::provide::discrete<T>(
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
  typename T = MODBUS_TYPE_DEFAULT,
  typename I = MODBUS_TYPE_BIND_INDEX>
::gos::atl::modbus::binding::result access(
    ::gos::atl::binding::reference<B, T, I>& binding,
    ::gos::atl::modbus::structures::Variable<T>& variable,
    ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& request,
    ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& response,
    const T& start,       // Start address for the modbus function
    const T& length) {    // Length of the buffer for the modbus function
  ::gos::atl::modbus::binding::result result;
  T address, first, last;
  I index;
  result = ::gos::atl::modbus::binding::detail::initialize<B, T, I>(
    binding, start, length, address, first, last, index);
  if (result == ::gos::atl::modbus::binding::result::included) {
    while (first < length && index < binding.count) {
      ::gos::atl::modbus::provide::registers<T>(
        variable, request, response, first, *(binding.pointers[index]));
      first++;
      index++;
    }
  }
  return result;
}

template<typename B, typename T = MODBUS_TYPE_DEFAULT>
::gos::atl::modbus::binding::result access(
  ::gos::atl::binding::barray::reference<B, T, MODBUS_TYPE_BIND_INDEX>& binding,
  ::gos::atl::modbus::structures::Variable<T>& variable,
  ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& request,
  ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& response,
  const T& start,       // Start address for the modbus function
  const T& length) {    // Length of the buffer for the modbus function
  ::gos::atl::modbus::binding::result result;
  MODBUS_TYPE_DEFAULT address, first, last;
  MODBUS_TYPE_BIND_INDEX index;
  result = ::gos::atl::modbus::binding::detail::initialize<
    B, T, MODBUS_TYPE_BIND_INDEX>(
    binding, start, length, address, first, last, index);
  if (result == ::gos::atl::modbus::binding::result::included) {
    while (first < length && index < binding.count) {
      ::gos::atl::modbus::provide::registers<T>(
        variable, request, response, first, binding.pointers[index]);
      first++;
      index++;
    }
  }
  return result;
}

template<typename B, typename T = MODBUS_TYPE_DEFAULT>
::gos::atl::modbus::binding::result assign(
  ::gos::atl::binding::reference<B, T, MODBUS_TYPE_BIND_INDEX>& binding,
  ::gos::atl::modbus::structures::Variable<T>& variable,
  ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& request,
  ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& response,
  const T& start,    // Start address for the modbus function
  const T& length,   // Length of the buffer for the modbus function
  MODBUS_TYPE_DEFAULT& address,
  MODBUS_TYPE_DEFAULT& first,
  MODBUS_TYPE_DEFAULT& last,
  MODBUS_TYPE_BIND_INDEX index) {
  ::gos::atl::modbus::binding::result result;
  result = ::gos::atl::modbus::binding::detail::initialize<
    B, T, MODBUS_TYPE_BIND_INDEX>(
    binding, start, length, address, first, last, index);
  if (result == ::gos::atl::modbus::binding::result::included) {
    while (first < length && index < binding.count) {
      *(binding.pointers[index]) = static_cast<B>(
        ::gos::atl::modbus::access::registers<T>(variable, request, first));
      first++;
      index++;
    }
  }
  return result;
}

template<typename B, typename T = MODBUS_TYPE_DEFAULT>
::gos::atl::modbus::binding::result assign(
  ::gos::atl::binding::barray::reference<B, T, MODBUS_TYPE_BIND_INDEX>& binding,
  ::gos::atl::modbus::structures::Variable<T>& variable,
  ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& request,
  ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& response,
  const T& start,    // Start address for the modbus function
  const T& length,   // Length of the buffer for the modbus function
  MODBUS_TYPE_DEFAULT& address,
  MODBUS_TYPE_DEFAULT& first,
  MODBUS_TYPE_DEFAULT& last,
  MODBUS_TYPE_BIND_INDEX index) {
  ::gos::atl::modbus::binding::result result;
  result = ::gos::atl::modbus::binding::detail::initialize<
    B, T, MODBUS_TYPE_BIND_INDEX>(
    binding, start, length, address, first, last, index);
  if (result == ::gos::atl::modbus::binding::result::included) {
    while (first < length && index < binding.count) {
      binding.pointers[index] = ::gos::atl::modbus::access::registers<T>(
          variable, request, first);
      first++;
      index++;
    }
  }
  return result;
}


}

namespace two {
template<typename B, typename T = MODBUS_TYPE_DEFAULT>
::gos::atl::modbus::binding::result access(
  ::gos::atl::binding::reference<B, T, MODBUS_TYPE_BIND_INDEX>& binding,
  ::gos::atl::modbus::structures::Variable<T>& variable,
  ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& request,
  ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& response,
  const T& start,       // Start address for the modbus function
  const T& length) {    // Length of the buffer for the modbus function
  ::gos::atl::modbus::binding::result result;
  MODBUS_TYPE_DEFAULT address, first, last;
  MODBUS_TYPE_BIND_INDEX index;
  result = ::gos::atl::modbus::binding::detail::initialize<
    B, T, MODBUS_TYPE_BIND_INDEX>(
    binding, start, length, address, first, last, index);
  if (result == ::gos::atl::modbus::binding::result::included) {
    while (first < length && index < binding.count) {
      ::gos::atl::modbus::provide::registers<T>(
        variable, request, response, first++,
        ::gos::atl::utility::number::part::first<T, B>(
          *(binding.pointers[index])));
      ::gos::atl::modbus::provide::registers<T>(
        variable, request, response, first++,
        ::gos::atl::utility::number::part::second<T, B>(
          *(binding.pointers[index])));
      index++;
    }
  }
  return result;
}

template<typename B, typename T = MODBUS_TYPE_DEFAULT >
::gos::atl::modbus::binding::result access(
  ::gos::atl::binding::barray::reference<B, T, MODBUS_TYPE_BIND_INDEX>& binding,
  ::gos::atl::modbus::structures::Variable<T>& variable,
  ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& request,
  ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& response,
  const T& start,       // Start address for the modbus function
  const T& length) {    // Length of the buffer for the modbus function
  ::gos::atl::modbus::binding::result result;
  MODBUS_TYPE_DEFAULT address, first, last;
  MODBUS_TYPE_BIND_INDEX index;
  result = ::gos::atl::modbus::binding::detail::initialize<
    B, T, MODBUS_TYPE_BIND_INDEX>(
    binding, start, length, address, first, last, index);
  if (result == ::gos::atl::modbus::binding::result::included) {
    while (first < length && index < binding.count) {
      ::gos::atl::modbus::provide::registers<T>(
        variable, request, response, first++,
        ::gos::atl::utility::number::part::first<T, B>(
          binding.pointers[index]));
      ::gos::atl::modbus::provide::registers<T>(
        variable, request, response, first++,
        ::gos::atl::utility::number::part::second<T, B>(
          binding.pointers[index]));
      index++;
    }
  }
  return result;
}

template<typename B, typename T = MODBUS_TYPE_DEFAULT> ::gos::atl::modbus::binding::result assign(
    ::gos::atl::binding::reference<B, T, MODBUS_TYPE_BIND_INDEX>& binding,
    ::gos::atl::modbus::structures::Variable<T>& variable,
    ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& request,
    ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& response,
    const T& start,    // Start address for the modbus function
    const T& length,   // Length of the buffer for the modbus function
    MODBUS_TYPE_DEFAULT& address,
    MODBUS_TYPE_DEFAULT& first,
    MODBUS_TYPE_DEFAULT& last,
    MODBUS_TYPE_BIND_INDEX index) {
  ::gos::atl::modbus::binding::result result;
  result = ::gos::atl::modbus::binding::detail::initialize<
    B, T, MODBUS_TYPE_BIND_INDEX>(
    binding, start, length, address, first, last, index);
  if (result == ::gos::atl::modbus::binding::result::included) {
    while (first < length && index < binding.count) {
      *(binding.pointers[index]) =
        ::gos::atl::utility::number::part::combine<T, B>(
        ::gos::atl::modbus::access::registers<T>(variable, request, first++),
        ::gos::atl::modbus::access::registers<T>(variable, request, first++));
      index++;
    }
  }
  return result;
}

template<typename B, typename T = MODBUS_TYPE_DEFAULT>
::gos::atl::modbus::binding::result assign(
  ::gos::atl::binding::barray::reference<B, T, MODBUS_TYPE_BIND_INDEX>& binding,
  ::gos::atl::modbus::structures::Variable<T>& variable,
  ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& request,
  ::gos::atl::buffer::Holder<T, MODBUS_TYPE_BUFFER>& response,
  const T& start,    // Start address for the modbus function
  const T& length,   // Length of the buffer for the modbus function
  MODBUS_TYPE_DEFAULT& address,
  MODBUS_TYPE_DEFAULT& first,
  MODBUS_TYPE_DEFAULT& last,
  MODBUS_TYPE_BIND_INDEX index) {
  ::gos::atl::modbus::binding::result result;
  result = ::gos::atl::modbus::binding::detail::initialize<
    B, T, MODBUS_TYPE_BIND_INDEX>(
    binding, start, length, address, first, last, index);
  if (result == ::gos::atl::modbus::binding::result::included) {
    while (first < length && index < binding.count) {
      binding.pointers[index] =
        ::gos::atl::utility::number::part::combine<T, B>(
          ::gos::atl::modbus::access::registers<T>(variable, request, first++),
          ::gos::atl::modbus::access::registers<T>(variable, request, first++));
      index++;
    }
  }
  return result;
}
}

} // binding namespace


}
}
}

#endif
