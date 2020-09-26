#pragma once

#include "kappa/input/abstractInput.hpp"
#include "okapi/api/filter/filter.hpp"
#include "okapi/api/filter/passthroughFilter.hpp"
#include <memory>

namespace kappa {

/*
 * It will assume you are calling your control loop at a constant interval.
 */

template <typename T>
class InputDifferentiator : public AbstractInput<T> {
public:
  // conversion = (# of controller iterations in target time unit) / (# of encoder ticks in target distance unit)
  InputDifferentiator(double iconversion, std::shared_ptr<AbstractInput<T>> iinput):
    InputDifferentiator(iconversion, std::make_unique<okapi::PassthroughFilter>(), iinput) {}

  // conversion = (# of controller iterations in target time unit) / (# of encoder ticks in target distance unit)
  InputDifferentiator(double iconversion, std::unique_ptr<okapi::Filter> ifilter, std::shared_ptr<AbstractInput<T>> iinput):
    input(iinput), filter(std::move(ifilter)), conversion(iconversion) {}


  virtual const T &get() override {
    const T &value = input->get();
    out = static_cast<T>(filter->filter(static_cast<double>(value - lastValue)) * conversion);
    lastValue = value;
    return out;
  }

  std::shared_ptr<AbstractInput<T>> getInput() const {
    return input;
  }

protected:
  std::shared_ptr<AbstractInput<T>> input{nullptr};
  std::unique_ptr<okapi::Filter> filter{nullptr};
  double conversion{0};

  T lastValue{0};
  T out{0};
};

extern template class InputDifferentiator<double>;

}
