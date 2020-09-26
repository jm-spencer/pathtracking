#pragma once

#include "kappa/input/simpleInput.hpp"
#include "okapi/api/device/rotarysensor/continuousRotarySensor.hpp"
#include <memory>

namespace kappa {

class OkapiInput : public SimpleInput<double> {
public:
  OkapiInput(std::shared_ptr<okapi::RotarySensor> iinput, double igearRatio = 1.0);

  std::shared_ptr<okapi::RotarySensor> getInput() const;

  virtual const double &get() override;

  virtual const double &getValue() const override;

protected:
  std::shared_ptr<okapi::RotarySensor> input{nullptr};
  double value{0};
  double gearRatio{1};
};

}
