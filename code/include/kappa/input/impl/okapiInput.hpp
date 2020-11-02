#pragma once

#include "kappa/input/simpleInput.hpp"
#include "okapi/api/device/rotarysensor/continuousRotarySensor.hpp"
#include <memory>

namespace kappa {

class OkapiInput : public SimpleInput<double> {
public:

  /**
   * Wrapper for any okapi rotary sensor
   * 
   * @param iinput pointer to okapi sensor
   * @param igearRatio a scalar to the value of the sensor. Useful for external gear ratios
   */
  OkapiInput(std::shared_ptr<okapi::RotarySensor> iinput, double igearRatio = 1.0);

  /**
   * get a pointer to the underlying object
   * 
   * @return pointer to the okapi sensor object
   */
  std::shared_ptr<okapi::RotarySensor> getInput() const;

  /**
   * get the reading value of the sensor
   * 
   * @return value
   */
  virtual const double &get() override;

  /**
   * get the last reading value of the sensor (no update)
   * 
   * @return value
   */
  virtual const double &getValue() const override;

protected:
  std::shared_ptr<okapi::RotarySensor> input;
  double gearRatio;
  double value{0};
};

}
