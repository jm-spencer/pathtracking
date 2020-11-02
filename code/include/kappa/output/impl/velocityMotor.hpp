#pragma once

#include "kappa/output/abstractOutput.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"


namespace kappa {

class VelocityMotor : public AbstractOutput<double> {
public:

  /**
   * Wrapper for an okapi motor (or motor group) which is controlled by
   * integrated velocity control
   *
   * @param imotor okapi motor object
   */
  VelocityMotor(std::shared_ptr<okapi::AbstractMotor> imotor);

  /**
   * Set target velocity, in units of rpm, to the integrated motor controller
   *
   * @param itarget target value
   */
  virtual void set(const double &itarget) override;

  /**
   * Gets underlying motor
   *
   * @return motor
   */
  std::shared_ptr<okapi::AbstractMotor> getMotor() const;

protected:
  std::shared_ptr<okapi::AbstractMotor> motor;

  double maxVelocity{0};
};

}
