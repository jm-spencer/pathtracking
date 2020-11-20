#pragma once

#include "kappa/input/abstractInput.hpp"
#include "okapi/api/filter/filter.hpp"
#include "okapi/api/filter/passthroughFilter.hpp"
#include "pros/rtos.hpp"
#include <memory>

namespace kappa {

template <typename T>
class InputDifferentiator : public AbstractInput<T> {
public:

  /**
   * Differentiates input data. eg. position to velocity or velocity to acceleration
   * Ensure this control look is polled at a constant interval
   * Assumes T can cast to double
   *
   * @param iconversion
   * (# of milliseconds in target time unit) / (# of encoder ticks in target distance unit)
   * @param iinput input for data
   */
  InputDifferentiator(double iconversion, std::shared_ptr<AbstractInput<T>> iinput):
    InputDifferentiator(iconversion, std::make_unique<okapi::PassthroughFilter>(), iinput) {}

  /**
   * @param iconversion
   * (# of milliseconds in target time unit) / (# of encoder ticks in target distance unit)
   * @param ifilter filter after differentiation
   * @param iinput input for data
   */
  InputDifferentiator(double iconversion, std::unique_ptr<okapi::Filter> ifilter, std::shared_ptr<AbstractInput<T>> iinput):
    input(iinput), filter(std::move(ifilter)), conversion(iconversion)
    {
      lastTime = pros::millis();
    }

  /**
   * Gets data from its input, differentiates, and returns it
   *
   * @return input data
   */
  virtual const T &get() override {
    const T &value = input->get();
    out = static_cast<T>(filter->filter(static_cast<double>((value - lastValue) / (pros::millis() - lastTime)) * conversion));
    lastValue = value;
    lastTime = pros::millis();
    return out;
  }

  /**
   * Gets input source
   *
   * @return input
   */
  std::shared_ptr<AbstractInput<T>> getInput() const {
    return input;
  }

protected:
  std::shared_ptr<AbstractInput<T>> input{nullptr};
  std::unique_ptr<okapi::Filter> filter{nullptr};
  double conversion{0};

  T lastValue;
  uint32_t lastTime{0};
  T out;
};

extern template class InputDifferentiator<double>;

}
