#ifndef _GOS_ARDUINO_TEMPLATE_LIBRARY_PID_H_
#define _GOS_ARDUINO_TEMPLATE_LIBRARY_PID_H_

#include <Arduino.h>

#include <gatltype.h>
#include <gatlutility.h>

namespace gos {
namespace atl {
namespace pid {

/* For reverse flip the signs for Kp, Ki, Kd */
template<typename I, typename O = I, typename P = I>
struct Parameter {
  ::gos::atl::type::range<O> Range;
  I Setpoint;
  P Time;
  P Kp;
  bool PonE;
};

template<typename T>
struct Tune {
  T Ki;
  T Kd;
};

template<typename T>
struct TimeTune {
  T Ti;
  T Td;
};

template<typename V>
struct Variable {
  V LastInput;
  V KiTimesTime;      /* Ki * sample time */
  V KdDividedByTime;  /* Kd / sample time */
  V OutputSum;
};

template<typename V>
struct Values {
  bool Mode;
  V Manual;
};

namespace time {
namespace milliseconds {
template<typename T> T Ki(const T& gain, const T& ti) {
  return gain / (ti / T(1000));
}
template<typename T> T Kd(const T& gain, const T& td) {
  return gain * (td / T(1000));
}
template<typename T> T Ti(const T& kp, const T& ki) {
  return T(1000) * kp / ki;
}
template<typename T> T Td(const T& kp, const T& kd) {
  return T(1000) * kd / kp;
}
template<typename V, typename I = V, typename O = V, typename P = V>
void tunings(
  Variable<V>& variable,
  const Parameter<I, O, P>& parameter,
  const P& ki,
  const P& kd) {
  variable.KiTimesTime = static_cast<V>(ki * (parameter.Time/ P(1000)));
  variable.KdDividedByTime = static_cast<V>(kd / (parameter.Time / P(1000)));
}
template<typename V, typename I = V, typename O = V, typename P = V>
void tunings(
  Variable<V>& variable,
  const Parameter<I, O, P>& parameter,
  const Tune<P>& tune) {
  tunings(variable, parameter, tune.Ki, tune.Kd);
}
template<typename V, typename I = V, typename O = V, typename P = V>
void tunings(
  Variable<V>& variable,
  const Parameter<I, O, P>& parameter,
  const TimeTune<P>& tune) {
  tunings(
    variable,
    parameter,
    Ki(parameter.Kp, tune.Ti),
    Kd(parameter.Kp, tune.Td));
}
}
namespace seconds {
template<typename T> T Ki(const T& gain, const T& ti) {
  return gain / ti;
}
template<typename T> T Kd(const T& gain, const T& td) {
  return gain * td;
}
template<typename T> T Ti(const T& kp, const T& ki) {
  return kp / ki;
}
template<typename T> T Td(const T& kp, const T& kd) {
  return kd / kp;
}
template<typename V, typename I = V, typename O = V, typename P = V>
void tunings(
  Variable<V>& variable,
  const Parameter<I, O, P>& parameter,
  const P& ki,
  const P& kd) {
  variable.KiTimesTime = static_cast<V>(ki * parameter.Time);
  variable.KdDividedByTime = static_cast<V>(kd / parameter.Time);
}
template<typename V, typename I = V, typename O = V, typename P = V>
void tunings(
  Variable<V>& variable,
  const Parameter<I, O, P>& parameter,
  const Tune<P>& tune) {
  tunings(variable, parameter, tune.Ki, tune.Kd);
}
template<typename V, typename I = V, typename O = V, typename P = V>
void tunings(
  Variable<V>& variable,
  const Parameter<I, O, P>& parameter,
  const TimeTune<P>& tune) {
  tunings(
    variable,
    parameter,
    Ki(parameter.Kp, tune.Ti),
    Kd(parameter.Kp, tune.Td));
}
}
namespace minutes {
template<typename T> T Ki(const T& gain, const T& ti) {
  return gain / (T(60) * ti);
}
template<typename T> T Kd(const T& gain, const T& td) {
  return gain * (T(60) * td);
}
template<typename T> T Ti(const T& kp, const T& ki) {
  return (kp / ki) / T(60);
}
template<typename T> T Td(const T& kp, const T& kd) {
  return (kd / kp) / T(60);
}
template<typename V, typename I = V, typename O = V, typename P = V>
void tunings(
  Variable<V>& variable,
  const Parameter<I, O, P>& parameter,
  const P& ki,
  const P& kd) {
  variable.KiTimesTime = static_cast<V>(ki * (P(60) * parameter.Time));
  variable.KdDividedByTime = static_cast<V>(kd / (P(60) * parameter.TimeMs));
}
template<typename V, typename I = V, typename O = V, typename P = V>
void tunings(
  Variable<V>& variable,
  const Parameter<I, O, P>& parameter,
  const Tune<P>& tune) {
  tunings(variable, parameter, tune.Ki, tune.Kd);
}
template<typename V, typename I = V, typename O = V, typename P = V>
void tunings(
  Variable<V>& variable,
  const Parameter<I, O, P>& parameter,
  const TimeTune<P>& tune) {
  tunings(
    variable,
    parameter,
    Ki(parameter.Kp, tune.Ti),
    Kd(parameter.Kp, tune.Td));
}
}
}

template<typename V, typename I = V, typename O = V> void initialize(
  Variable<V>& variable,
  const ::gos::atl::type::range<O>& range,
  const I& input = I(),
  const O& output = O()) {
  variable.LastInput = static_cast<V>(input);
  variable.OutputSum =
    static_cast<V>(::gos::atl::utility::range::restrict<O>(output, range));
}

template<typename I, typename O = I, typename P = I, typename V = I>
O compute(
  const I& input,
  Variable<V>& variable,
  const Parameter<I, O, P>& parameter) {
  /* Calculate error and delta input */
  V error = static_cast<V>(parameter.Setpoint - input);
  V i = static_cast<V>(input);
  V di = i - variable.LastInput;
  variable.LastInput = i;
  variable.OutputSum += error * variable.KiTimesTime;
  if (!parameter.PonE) {
    variable.OutputSum -= di * static_cast<V>(parameter.Kp);
  }
  variable.OutputSum = static_cast<V>(::gos::atl::utility::range::restrict<O>(
    static_cast<O>(variable.OutputSum), parameter.Range));
  V output = parameter.PonE ? static_cast<V>(parameter.Kp) * error : V();
  output += variable.OutputSum - variable.KdDividedByTime * di;
  return ::gos::atl::utility::range::restrict<O>(
    static_cast<O>(output),
    parameter.Range);
}

}
}
}

#endif /* _GOS_ARDUINO_TEMPLATE_LIBRARY_PID_H_ */
