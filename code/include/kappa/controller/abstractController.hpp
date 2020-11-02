#pragma once

#include "kappa/input/abstractInput.hpp"
#include "kappa/output/abstractOutput.hpp"
#include "pros/rtos.hpp"
#include <cfloat>


namespace kappa {

template <typename IN, typename TARGET, typename OUT>
class AbstractController : public AbstractOutput<TARGET> {
public:

  /**
   * A controller, which uses some sensory data given by its input to determine
   * an output that will drive the system to the desired target.
   */
  AbstractController() {}

  /**
   * Sets the target for the controller
   *
   * @param itarget the new target
   */
  virtual void setTarget(const TARGET &itarget) = 0;

  /**
   * Sets the target for the controller
   *
   * @param itarget the new target
   */
  virtual void set(const TARGET &itarget) override {
      setTarget(itarget);
  }

  /**
   * Returns the current controller target
   *
   * @return the current target
   */
  virtual TARGET getTarget() const {
    return target;
  };

  /**
   * steps the controller, calculating a new output value from the current input
   *
   * @param ireading the new sensor reading
   * @return the calculated output signal
   */
  virtual OUT step(IN ireading) = 0;

  /**
   * Returns the last calculated output signal without updating the controller
   *
   * @return the last calculated output signal
   */
  virtual OUT getOutput() const {
    return output;
  };

  /**
   * Returns the last sensory input
   *
   * @return the last sensory input
   */
  virtual IN getLastInput() const {
    return lastReading;
  }

  /**
   * Returns the last calculated error
   *
   * @return the last calculated error
   */
  virtual IN getError() const {
    return error;
  }

  /**
   * Tests if the robot is settled at the target position,
   * using okapilib's ssettledUtil
   */
  virtual bool isSettled() = 0;

  /**
   * Waits until the robot is settled
   *
   * @param timestep polling interval in ms
   */
  virtual void waitUntilSettled(uint32_t timestep = 10) {
    while(!isSettled()){
      pros::delay(timestep);
    }
  }

  /**
   * Resets the controller to behavior immediately after construction
   * (like resetting "last iteration" values)
   */
  virtual void reset() = 0;

  /**
   * Disable the controller, making its output always 0
   *
   * @param iisDisabled if true, disable, if false, enable
   */
  virtual void disable(bool iisDisabled) = 0;

  /**
   * Check if the controller is disabled
   *
   * @return if the controller is disabled
   */
  virtual bool isDisabled() const {
      return disabled;
  }

protected:
  TARGET target{0};
  IN lastReading{0};
  IN error{0};

  OUT output{0};

  bool disabled{false};
};

}
