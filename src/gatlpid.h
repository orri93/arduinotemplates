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
  P TimeMs;
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

template<typename T> T Ki(const T& gain, const T& ti) {
#ifndef GATL_PID_TUNING_TIME_IN_MIN
  return gain / ti;
#else
  return gain / (T(60) * ti);
#endif
}

template<typename T> T Kd(const T& gain, const T& td) {
#ifndef GATL_PID_TUNING_TIME_IN_MIN
  return gain * td;
#else
  return gain * (T(60) * td);
#endif
}

template<typename T> T Ti(const T& kp, const T& ki) {
#ifndef GATL_PID_TUNING_TIME_IN_MIN
  return kp / ki;
#else
  return (kp / ki) / T(60);
#endif
}

template<typename T> T Td(const T& kp, const T& kd) {
#ifndef GATL_PID_TUNING_TIME_IN_MIN
  return kd / kp;
#else
  return (kd / kp) / T(60);
#endif
}

template<typename V, typename I = V, typename O = V, typename P = V>
void tunings(
  Variable<V>& variable,
  const Parameter<I, O, P>& parameter,
  const P& ki,
  const P& kd) {
#ifndef GATL_PID_TUNING_IN_MS
  variable.KiTimesTime = static_cast<V>(ki * (parameter.TimeMs / P(1000)));
  variable.KdDividedByTime = static_cast<V>(kd / (parameter.TimeMs / P(1000)));
#else
  variable.KiTimesTime = static_cast<V>(ki * parameter.TimeMs);
  variable.KdDividedByTime = static_cast<V>(kd / parameter.TimeMs);
#endif
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

template<typename V, typename I = V, typename O = V> void initialize(
  Variable<V>& variable,
  const ::gos::atl::type::range<O>& range,
  const I& input = I(),
  const O& output = O()) {
  variable.LastInput = static_cast<V>(input);
  variable.OutputSum =
    static_cast<V>(::gos::atl::utility::range::restrict(output, range));
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
#ifndef GATL_PID_TUNING_IN_MS
  variable.OutputSum += error * variable.KiTimesTime;
#else
  variable.OutputSum += error * parameter.KiTimesTime / V(1000);
#endif
  if (!parameter.PonE) {
    variable.OutputSum -= di * static_cast<V>(parameter.Kp);
  }
  variable.OutputSum = ::gos::atl::utility::range::restrict(
    variable.OutputSum, parameter.Range);
  V output = parameter.PonE ? static_cast<V>(parameter.Kp) * error : V();
#ifndef GATL_PID_TUNING_IN_MS
  output += variable.OutputSum - variable.KdDividedByTime * di;
#else
  output += variable.OutputSum - V(1000) * parameter.KdDividedByTime * di;
#endif
  return ::gos::atl::utility::range::restrict(
    static_cast<O>(output),
    parameter.Range);
}

}
}
}

#endif /* _GOS_ARDUINO_TEMPLATE_LIBRARY_PID_H_ */
