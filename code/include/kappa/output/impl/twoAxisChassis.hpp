#pragma once

#include "kappa/output/abstractOutput.hpp"
#include <tuple>
#include <memory>


namespace kappa {

class TwoAxisChassis : public AbstractOutput<std::tuple<double,double>> {
public:

  /**
   * Chassis object for a chassis with 2 degrees of freedom (nonholonomic)
   * Converts an target signal of (forward, ccw)
   * to an signal of (left, right)
   * for an skid-steer like chassis
   *
   * @param ichassis array of motors (or motor groups) in format (left, right)
   */
  TwoAxisChassis(std::shared_ptr<AbstractOutput<std::array<double,2>>> ichassis);

  /**
   * Chassis object for a chassis with 2 degrees of freedom (nonholonomic)
   * Converts an target signal of (forward, ccw) in units of (in/s, rad/s)
   * to an signal of (left, right) in units of rpm
   * for an skid-steer like chassis
   *
   * @param iwheelDiameter diameter of wheel in inches
   * @param ichassisWidth distance between parallel wheels in inches
   * @param ichassis array of motors (or motor groups) in format (left, right)
   */
  TwoAxisChassis(double iwheelDiameter, double ichassisWidth, std::shared_ptr<AbstractOutput<std::array<double,2>>> ichassis);

  /**
   * Calculates target values for each motor and sets their respective targets
   *
   * @param itarget target values
   */
  virtual void set(const std::tuple<double,double> &itarget) override;

  /**
   * Gets output
   *
   * @return output
   */
  std::shared_ptr<AbstractOutput<std::array<double,2>>> getOutput() const;

protected:
  std::shared_ptr<AbstractOutput<std::array<double,2>>> chassis{nullptr};
  double linearScalar{0};
  double angularScalar{0};
};

}
