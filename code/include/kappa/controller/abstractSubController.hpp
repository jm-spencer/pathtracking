#pragma once

#include "kappa/input/abstractInput.hpp"
#include "kappa/output/abstractOutput.hpp"
#include <memory>
#include <cfloat>


namespace kappa {

template <typename IN, typename TARGET, typename OUT>
class AbstractSubController : public AbstractOutput<TARGET> {
public:

  /**
   * A subcontroller, whose target is determined by the output of another
   * controller and run inline with that other controller.
   *
   * @param iinput input value for the controller
   * @param ioutput output for the target signal
   */
  AbstractSubController(std::shared_ptr<AbstractInput<IN>> iinput, std::shared_ptr<AbstractOutput<OUT>> ioutputDevice):
    input(iinput), outputDevice(ioutputDevice) {}

  /**
   * Returns the current controller target
   *
   * @return the current target
   */
  virtual TARGET getTarget() const {
    return target;
  };

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
   * Resets the controller to behavior immediately after construction
   * (like resetting "last iteration" values)
   */
  virtual void reset() = 0;

protected:
  TARGET target{0};
  IN lastReading{0};
  IN error{0};

  OUT output{0};

  std::shared_ptr<AbstractInput<IN>> input{nullptr};
  std::shared_ptr<AbstractOutput<OUT>> outputDevice{nullptr};
};

}
