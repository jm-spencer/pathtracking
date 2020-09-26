#pragma once

#include "kappa/input/simpleInput.hpp"

namespace kappa {

class TimeInput : public SimpleInput<double> {
public:
  TimeInput();

  virtual const double &get() override;

  virtual const double &getValue() const override;

protected:
  double value{0};
};

}
