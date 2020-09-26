#pragma once

#include "kappa/input/simpleInput.hpp"
#include "pros/imu.hpp"
#include <memory>


namespace kappa {

class ImuInput : public SimpleInput<double> {
public:
  ImuInput(const std::uint8_t port, bool ccw = true); // if true, the imu will return ccw rotation as positive

  std::int32_t calibrate() const;

  std::shared_ptr<pros::Imu> getInput() const;

  virtual const double &get() override;

  virtual const double &getValue() const override;

protected:
  std::shared_ptr<pros::Imu> input;
  double coef;
  double value{0};
};

}
