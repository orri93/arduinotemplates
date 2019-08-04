#ifndef _GOS_ARDUINO_TEMPLATE_LIBRARY_SORT_H_
#define _GOS_ARDUINO_TEMPLATE_LIBRARY_SORT_H_

#include <Arduino.h>

namespace gos {
namespace atl {
namespace sort {

template<typename T, typename C = uint8_t>
void insertion(T* values, const C& count) {
  T t;
  C i, j;
  for (i = 1; i < count; i++) {
    for (j = i; j > 0 && (values[j - 1] > values[j]); j--) {
      t = values[j - 1];
      values[j - 1] = values[j];
      values[j] = t;
    }
  }
}

template<typename T, typename R, typename C = uint8_t>
void insertion(const T* values, R* references, const C& count) {
  R t;
  C i, j;
  for (i = 1; i < count; i++) {
    j = i;
    while (j > 0 && (values[references[j - 1]] > values[references[j]]))
    {
      t = references[j - 1];
      references[j - 1] = references[j];
      references[j] = t;
      j--;
    }
  }
}

}
}
}
#endif /* _GOS_ARDUINO_TEMPLATE_LIBRARY_SORT_H_ */
