#pragma once

#include "kappa/input/abstractInput.hpp"
#include "kappa/output/abstractOutput.hpp"
#include <memory>
#include <cfloat>


namespace kappa {

template <typename IN, typename TARGET, typename OUT>
class AbstractSubController : public AbstractOutput<TARGET> {
public:
  AbstractSubController(std::shared_ptr<AbstractInput<IN>> iinput, std::shared_ptr<AbstractOutput<OUT>> ioutputDevice, OUT ioutputMin = std::numeric_limits<OUT>::lowest(), OUT ioutputMax = std::numeric_limits<OUT>::max()):
    input(iinput), outputDevice(ioutputDevice), outputMin(ioutputMin), outputMax(ioutputMax) {}

  virtual TARGET getTarget() const {
    return target;
  };

  virtual OUT getOutput() const {
    return output;
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

  virtual void reset() = 0;

protected:
  TARGET target{0};
  IN lastReading{0};
  IN error{0};

  OUT output{0};
  OUT outputMax{std::numeric_limits<OUT>::max()};
  OUT outputMin{std::numeric_limits<OUT>::lowest()};

  std::shared_ptr<AbstractInput<IN>> input{nullptr};
  std::shared_ptr<AbstractOutput<OUT>> outputDevice{nullptr};
};

}
