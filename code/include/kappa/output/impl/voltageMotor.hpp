#pragma once

#include "kappa/output/abstractOutput.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"


namespace kappa {

class VoltageMotor : public AbstractOutput<double> {
public:
  VoltageMotor(std::shared_ptr<okapi::AbstractMotor> imotor);

  /**
   * in units of mV
   */
  virtual void set(const double &itarget) override;

  std::shared_ptr<okapi::AbstractMotor> getMotor() const;

protected:
  std::shared_ptr<okapi::AbstractMotor> motor;
};

}
