#pragma once

#include "kappa/output/abstractOutput.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"


namespace kappa {

class VoltageMotor : public AbstractOutput<double> {
public:

  /**
   * Wrapper for an okapi motor (or motor group) which is controlled by
   * pwm (pulse-width-modulation) voltage
   *
   * @param imotor okapi motor object
   */
  VoltageMotor(std::shared_ptr<okapi::AbstractMotor> imotor);

  /**
   * Set voltage, in units of mV (-12000,12000), to the motor
   *
   * @param itarget voltage
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
};

}
