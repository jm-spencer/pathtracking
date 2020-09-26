#pragma once

#include "kappa/input/abstractInput.hpp"
#include "kappa/output/abstractOutput.hpp"
#include "pros/rtos.hpp"
#include <cfloat>


namespace kappa {

template <typename IN, typename TARGET, typename OUT>
class AbstractController : public AbstractOutput<TARGET> {
public:
  AbstractController(OUT ioutputMin = std::numeric_limits<OUT>::lowest(), OUT ioutputMax = std::numeric_limits<OUT>::max()):
    outputMin(ioutputMin), outputMax(ioutputMax) {}

  virtual void setTarget(const TARGET &itarget) = 0;

  virtual void set(const TARGET &itarget) override {
      setTarget(itarget);
  }

  virtual TARGET getTarget() const {
    return target;
  };

  virtual OUT step(IN ireading) = 0;

  virtual OUT getOutput() const {
    return isDisabled() ? 0 : output;
  };

  virtual void setOutputLimits(OUT imin, OUT imax) {
    outputMin = imin;
    outputMax = imax;
  }

  virtual OUT getMinOutput() const {
    return outputMin;
  }

  virtual OUT getMaxOutput() const {
    return outputMax;
  }

  virtual IN getLastInput() const {
    return lastReading;
  }

  virtual IN getError() const {
    return error;
  }

  virtual bool isSettled() = 0;

  virtual void waitUntilSettled(uint32_t timestep = 10) {
    while(!isSettled()){
      pros::delay(timestep);
    }
  }

  virtual void reset() = 0;

  virtual void disable(bool iisDisabled) = 0;

  virtual bool isDisabled() const {
      return disabled;
  }

protected:
  TARGET target{0};
  IN lastReading{0};
  IN error{0};

  OUT output{0};
  OUT outputMax{std::numeric_limits<OUT>::max()};
  OUT outputMin{std::numeric_limits<OUT>::lowest()};

  bool disabled{false};
};

}
