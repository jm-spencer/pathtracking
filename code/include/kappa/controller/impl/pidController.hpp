#pragma once

#include "kappa/controller/abstractController.hpp"
#include "okapi/api/control/util/settledUtil.hpp"
#include "okapi/api/filter/filter.hpp"
#include "okapi/api/filter/passthroughFilter.hpp"
#include "okapi/impl/util/timeUtilFactory.hpp"
#include <memory>

namespace kappa {

class PidController : public AbstractController<double, double, double> {
public:

  struct Gains {
    double kP;
    double kI;
    double kD;
    double kF;
  };

  /**
   * A standard implementation of a PID controller with a feedforward term
   *
   * @param igains gains for the controller
   * @param itimeUtil see okapi::TimeUtil docs, settledUtil is utilized here
   * @param iderivativeFilter filter for the derivative of the error
   */
  PidController(Gains igains,
                const okapi::TimeUtil &itimeUtil = okapi::TimeUtilFactory::createDefault(),
                std::unique_ptr<okapi::Filter> iderivativeFilter = std::make_unique<okapi::PassthroughFilter>());

  /**
   * @param igains gains for the controller
   * @param ioutputMin minimum value for controller output
   * @param ioutputMax maximum value for controller output
   * @param itimeUtil see okapi::TimeUtil docs
   * @param iderivativeFilter filter for the derivative of the error
   */
  PidController(Gains igains,
                double ioutputMin,
                double ioutputMax,
                const okapi::TimeUtil &itimeUtil = okapi::TimeUtilFactory::createDefault(),
                std::unique_ptr<okapi::Filter> iderivativeFilter = std::make_unique<okapi::PassthroughFilter>());

  /**
   * Sets the target value of the controller
   *
   * @param itarget the new target
   */
  virtual void setTarget(const double &itarget) override;

  /**
   * Set the output limits that the output will be clamped between
   *
   * @param imin minimum output value
   * @param imax maximum output value
   */
  virtual void setOutputLimits(double imin, double imax);

  /**
   * Get the minimum output limit
   *
   * @return minimum output value
   */
  virtual double getMinOutput() const;

  /**
   * Get the maximum output limit
   *
   * @return maximum output value
   */
  virtual double getMaxOutput() const;

  /**
   * Iterates the controller
   *
   * @param ireading new sensor value
   * @return new output value
   */
  virtual double step(double ireading) override;

  /**
   * Checks if the controller is settled at the target point.
   * Uses okapi's settledUtil from inside timeUtil
   *
   * @return true if settled
   */
  virtual bool isSettled() override;

  /**
   * Resets the controller to behavior immediately after construction
   * (like reseting "last iteration" values)
   */
  virtual void reset() override;

  /**
   * Disabled the controller. While set to true, the controller always returns 0
   *
   * @param iisDisabled set to true to disable, false to resume
   */
  virtual void disable(bool iisDisabled) override;

  /**
   * Get the gains of the controller
   *
   * @return gains
   */
  Gains getGains() const;

  /**
   * Set the gains of the controller
   *
   * @param igains gains
   */
  void setGains(Gains igains);

protected:
  Gains gains;
  std::unique_ptr<okapi::SettledUtil> settledUtil{nullptr};
  std::unique_ptr<okapi::Filter> derivativeFilter{nullptr};

  double outputMax{DBL_MAX};
  double outputMin{-DBL_MAX};

  double lastError{0};
  double integral{0};
  double derivative{0};
};

}
