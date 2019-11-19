#ifndef _GOS_ARDUINO_TEMPLATE_LIBRARY_TYPE_H_
#define _GOS_ARDUINO_TEMPLATE_LIBRARY_TYPE_H_

#include <Arduino.h>

namespace gos {
namespace atl {
namespace type {

template<typename T> struct range {
  T lowest;
  T highest;
};

template<typename T> range<T> make_range(const T& lowest, const T& highest) {
  range<T> range;
  range.lowest = lowest;
  range.highest = highest;
  return range;
}

template<typename T>
class optional {
public:
  friend bool operator == (const optional<T>& rhs, const optional<T>& lhs);
  friend bool operator =! (const optional<T>& rhs, const optional<T>& lhs);
#ifdef NOT_USED
  friend bool operator > (const optional<T>& rhs, const optional<T>& lhs);
  friend bool operator >= (const optional<T>& rhs, const optional<T>& lhs);
  friend bool operator < (const optional<T>& rhs, const optional<T>& lhs);
  friend bool operator <= (const optional<T>& rhs, const optional<T>& lhs);
#endif
  optional() : p_(nullptr) {}
  optional(const T& value) : p_(new T(value)) {}
  optional(const optional& optional) = delete;
  optional(optional&& optional) : p_(optional.p_) {
    if (p_ != nullptr) {
      optional.p_ = nullptr;
    }
  }
  optional& operator=(const optional& optional) = delete;
  optional& operator=(optional&& optional) {
    if (this != &optional) {
      if ((p_ = optional.p_) != nullptr) {
        optional.p_ = nullptr;
      };
    }
    return *this;
  }
  optional& operator=(const T& value) {
    assign(value);
  }
  bool operator==(const T& value) const {
    return isequal(value);
  }
  bool operator!=(const T& value) const {
    return isunequal(value);
  }
  operator bool() const {
    return is();
  }
  operator T() const {
    return get();
  }
  operator T* () const {
    return p_;
  }
  bool isequal(const T& value) const {
    return (p_ != nullptr) ? (*p_ == value) : false;
  }
  bool isunequal(const T& value) const {
    return p_ != nullptr ? (*p_ != value) : true;
  }
  void assign(const T& value) {
    if (p_ != nullptr) {
      *p_ = value;
    } else {
      p_ = new T(value);
    }
  }
  void clear() {
    delete p_;
    p_ = nullptr;
  }
  bool is() const {
    return p_ != nullptr;
  }
  T* pointer() const {
    return p_;
  }
  T get() const {
    return p_ != nullptr ? *p_ : T();
  }
protected:
  T* p_;
};

}
}
}

template<typename T> bool operator==(
  const ::gos::atl::type::optional<T>& lhs,
  const ::gos::atl::type::optional<T>& rhs) {
  return lhs.p_ != nullptr && rhs.p_ != nullptr ? (*lhs.p_ == *rhs.p_) : false;
}
template<typename T> bool operator!=(
  const ::gos::atl::type::optional<T>& lhs,
  const ::gos::atl::type::optional<T>& rhs) {
  if (lhs.p_ != nullptr && rhs.p_ != nullptr) {
    return *lhs.p_ != *rhs.p_;
  } else if (lhs.p_ != nullptr || rhs.p_ != nullptr) {
    return true;
  } else {
    return false;
  }
}

#endif
