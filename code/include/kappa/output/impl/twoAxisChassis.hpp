#pragma once

#include "kappa/output/abstractOutput.hpp"
#include <tuple>
#include <memory>


namespace kappa {

/**
 *  controller input should be of the format (linear, angular),
 *  linear is in the units in/s, and angular in rad/s, and
 *  outputs are assumed to be in rpm.
 *  output array follows left, right motors
 */

class TwoAxisChassis : public AbstractOutput<std::tuple<double,double>> {
public:
  TwoAxisChassis(std::shared_ptr<AbstractOutput<std::array<double,2>>> ichassis);

  TwoAxisChassis(double iwheelDiameter, double ichassisWidth, std::shared_ptr<AbstractOutput<std::array<double,2>>> ichassis);

  virtual void set(const std::tuple<double,double> &itarget) override;

  std::shared_ptr<AbstractOutput<std::array<double,2>>> getOutput() const;

protected:
  std::shared_ptr<AbstractOutput<std::array<double,2>>> chassis{nullptr};
  double linearScalar{0};
  double angularScalar{0};
};

}
