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
  P Ki;
  P Kd;
  bool PonE;
};

template<typename V>
struct Variable {
  V LastInput;
  V KiTimesTime;      /* Ki * sample time */
  V KdDividedByTime;  /* Kd / sample time */
  V OutputSum;
};

template<typename V, typename I = V, typename O = V, typename P = V>
void tunings(Variable<V>& variable, const Parameter<I, O, P>& parameter) {
#ifdef GATL_PID_TUNING_IN_MS
  variable.KiTimesTime = static_cast<V>(parameter.Ki * parameter.TimeMs);
  variable.KdDividedByTime = static_cast<V>(parameter.Kd / parameter.TimeMs);
#else
  variable.KiTimesTime = static_cast<V>(parameter.Ki * (parameter.TimeMs / P(1000)));
  variable.KdDividedByTime = static_cast<V>(parameter.Kd / (parameter.TimeMs / P(1000)));
#endif
}

template<typename V, typename I = V, typename O = V> void initialize(
  Variable<V>& variable,
  const ::gos::atl::type::range<O>& range,
  const I& input = I(),
  const O& output = O()) {
  variable.LastInput = static_cast<V>(input);
  variable.OutputSum = static_cast<V>(::gos::atl::utility::restrict(output, range));
}

template<typename I, typename O = I, typename P = I, typename V = I>
O compute(const I& input, Variable<V>& variable, const Parameter<I, O, P>& parameter) {
  /* Calculate error and delta input */
  V error = static_cast<V>(parameter.Setpoint - input);
  V i = static_cast<V>(input);
  V di = i - variable.LastInput;
  variable.LastInput = i;
#ifdef GATL_PID_TUNING_IN_MS
  variable.OutputSum += error * parameter.KiTimesTime / V(1000);
#else
  variable.OutputSum += error * variable.KiTimesTime;
#endif
  if (!parameter.PonE) {
    variable.OutputSum -= di * static_cast<V>(parameter.Kp);
  }
  variable.OutputSum = ::gos::atl::utility::restrict(
    variable.OutputSum, parameter.Range);
  V output = parameter.PonE ? static_cast<V>(parameter.Kp) * error : V();
#ifdef GATL_PID_TUNING_IN_MS
  output += variable.OutputSum - V(1000) * parameter.KdDividedByTime * di;
#else
  output += variable.OutputSum - variable.KdDividedByTime * di;
#endif
  return ::gos::atl::utility::restrict(static_cast<O>(output), parameter.Range);
}

}
}
}

#endif /* _GOS_ARDUINO_TEMPLATE_LIBRARY_PID_H_ */
