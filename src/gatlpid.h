#ifndef _GOS_ARDUINO_TEMPLATE_LIBRARY_PID_H_
#define _GOS_ARDUINO_TEMPLATE_LIBRARY_PID_H_

#include <Arduino.h>

#ifdef GATL_PID_DEBUG
extern bool gatl_pid_debug;
#endif

namespace gos {
namespace atl {
namespace pid {

template<typename I, typename O = I, typename P = I>
struct Parameter {
  I Setpoint;
  O Minimum;
  O Maximum;
  /* For reverse flip the signs for Kp, Ki, Kd */
  P Kp;
  P KiTimesTime;      /* Ki * sample time in ms - for Ki use KiT / 1000 */
  P KdDividedByTime;  /* Kd / sample time in ms - for Kd use 1000 * KdT */
  bool PonE;
};

template<typename I, typename O = I, typename P = I>
void initialize(
  Parameter<I, O, P>& parameter,
  const O& minimum,
  const O& maximum,
  const P& timems,
  const P& kp,
  const P& ki,
  const P& kd,
  const bool& pone = false) {
  parameter.Setpoint = I();
  parameter.Minimum = minimum;
  parameter.Maximum = maximum;
  parameter.Kp = kp;
  parameter.KiTimesTime = ki * timems;
  parameter.KdDividedByTime = kd / timems;
  parameter.PonE = pone;
}


template<typename I, typename O = I>
struct Variable {
  I LastInput;
  O OutputSum;
};

template<typename I, typename O = I, typename P = I>
void initialize(
  Variable<I, O>& variables,
  const Parameter<I, O, P>& parameter,
  const I& input = I(),
  const O& output = O()) {
  variables.LastInput = input;
  if (output >= parameter.Minimum && output <= parameter.Maximum) {
    variables.OutputSum = output;
  } else if (output > parameter.Maximum) {
    variables.OutputSum = parameter.Maximum;
  } else {
    variables.OutputSum = parameter.Minimum;
  }
}

#ifdef GATL_PID_DEBUG
template<typename Q> void show(const char* text, const Q& q) {
  if (gatl_pid_debug) {
    std::cout << text << static_cast<float>(q) << std::endl;
  }
}
#define GATL_PID_DEBUG_SHOW(n,x) show(n,x)
#else
#define GATL_PID_DEBUG_SHOW(n,x)
#endif

template<typename I, typename O = I, typename P = I>
O compute(const I& input, Variable<I, O>& variable, const Parameter<I, O, P>& parameter) {
  GATL_PID_DEBUG_SHOW("Input: ", input);
  I error = parameter.Setpoint - input;
  GATL_PID_DEBUG_SHOW("Error: ", error);
  I di = input - variable.LastInput;
  GATL_PID_DEBUG_SHOW("Delta input: ", di);
  variable.LastInput = input;
  I kits = static_cast<I>(parameter.KiTimesTime) / I(1000);
  GATL_PID_DEBUG_SHOW("Ki * time(s): ", kits);
  GATL_PID_DEBUG_SHOW("Output sum: ", variable.OutputSum);
  variable.OutputSum += static_cast<O>(error * (static_cast<I>(parameter.KiTimesTime) / I(1000)));
  GATL_PID_DEBUG_SHOW("Output sum: ", variable.OutputSum);
  I kddts = I(1000) * static_cast<I>(parameter.KdDividedByTime);
  GATL_PID_DEBUG_SHOW("Kd / time(s): ", kddts);
  if (!parameter.PonE) {
    variable.OutputSum -= static_cast<O>(static_cast<I>(parameter.Kp) * di);
  }
  GATL_PID_DEBUG_SHOW("Output sum: ", variable.OutputSum);
  if (variable.OutputSum > parameter.Maximum) {
    variable.OutputSum = parameter.Maximum;
  }
  else if (variable.OutputSum < parameter.Minimum) {
    variable.OutputSum = parameter.Minimum;
  }
  GATL_PID_DEBUG_SHOW("Output sum: ", variable.OutputSum);
  O output = parameter.PonE ? static_cast<O>(static_cast<I>(parameter.Kp) * error) : O();
  GATL_PID_DEBUG_SHOW("Output: ", output);
  output += variable.OutputSum - static_cast<O>(I(1000) * static_cast<I>(parameter.KdDividedByTime) * di);
  GATL_PID_DEBUG_SHOW("Output: ", output);
  if (output >= parameter.Minimum && output <= parameter.Maximum) {
    return output;
  } else if (output > parameter.Maximum) {
    return parameter.Maximum;
  } else {
    return parameter.Minimum;
  }
}

}
}
}

#endif /* _GOS_ARDUINO_TEMPLATE_LIBRARY_PID_H_ */
