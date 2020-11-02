#pragma once

#include "kappa/input/simpleInput.hpp"

namespace kappa {

class TimeInput : public SimpleInput<double> {
public:

  /**
   * Returns the time. Mostly a dummy object for testing
   */
  TimeInput();

  /**
   * get the time
   * 
   * @return system time (in mS)
   */
  virtual const double &get() override;

  /**
   * get the last time get() was called
   * 
   * @return last system time (in mS)
   */
  virtual const double &getValue() const override;

protected:
  double value{0};
};

}
