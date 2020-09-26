#pragma once

#include "kappa/output/abstractOutput.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"


namespace kappa {

class VelocityMotor : public AbstractOutput<double> {
public:
  VelocityMotor(std::shared_ptr<okapi::AbstractMotor> imotor);

  /**
   * in units of rpm
   */
  virtual void set(const double &itarget) override;

  std::shared_ptr<okapi::AbstractMotor> getMotor() const;

protected:
  std::shared_ptr<okapi::AbstractMotor> motor;

  double maxVelocity{0};
};

}
