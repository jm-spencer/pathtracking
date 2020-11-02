#pragma once

#include "kappa/input/simpleInput.hpp"
#include "pros/imu.hpp"
#include <memory>


namespace kappa {

class ImuInput : public SimpleInput<double> {
public:

  /**
   * Wrapper for an IMU
   * Automatically starts calibration upon construction
   * 
   * @param port port on v5 brain
   * @param ccw if true, counts ccw rotation as positive, else cw rotation 
   * is positive. true by default
   */
  ImuInput(const std::uint8_t port, bool ccw = true);

  /**
   * Calibrates the IMU
   * 
   * @return 1 if the operation was successful or PROS_ERR if the operation
	 * failed, setting errno. See pros::Imu::reset for errno information
   */
  std::int32_t calibrate() const;

  /**
   * get a pointer to the imu object
   * 
   * @return pointer to the imu object
   */
  std::shared_ptr<pros::Imu> getInput() const;

  /**
   * get the reading value of the imu
   * 
   * @return value
   */
  virtual const double &get() override;

  /**
   * get the last reading value of the imu (no update)
   * 
   * @return value
   */
  virtual const double &getValue() const override;

protected:
  std::shared_ptr<pros::Imu> input;
  double coef;
  double value{0};
};

}
