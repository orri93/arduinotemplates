#ifndef _GOS_ARDUINO_TEMPLATE_LIBRARY_BUFFER_H_
#define _GOS_ARDUINO_TEMPLATE_LIBRARY_BUFFER_H_

#include <Arduino.h>

#ifndef GOS_ARDUINO_TEMPLATE_LIBRARY_BUFFER_SIZE
#define GOS_ARDUINO_TEMPLATE_LIBRARY_BUFFER_SIZE           11
#endif

namespace gos {
namespace atl {

namespace buffer {
template<typename S = uint8_t> class Holder {
public:
  Holder(const S& size = GOS_ARDUINO_TEMPLATE_LIBRARY_BUFFER_SIZE) : Size(size) {
    Buffer = (char*)(malloc(size));
  }
  Holder(const char* literal, const S& size) : Buffer(const_cast<char*>(literal)), Size(size - 1) {
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
  char* Buffer;
  S Size;
};
}

}
}

#endif /* _GOS_ARDUINO_TEMPLATE_LIBRARY_BUFFER_H_ */
