#pragma once

#include "kappa/input/abstractInput.hpp"
#include "okapi/api/filter/filter.hpp"
#include "okapi/api/filter/passthroughFilter.hpp"
#include <memory>

namespace kappa {

template <typename T>
class InputFilter : public AbstractInput<T> {
public:

  InputFilter(std::unique_ptr<okapi::Filter> ifilter, std::shared_ptr<AbstractInput<T>> iinput):
    input(iinput), filter(std::move(ifilter)) {}


  virtual const T &get() override {
    out = static_cast<T>(filter->filter(static_cast<double>(input->get())));
    return out;
  }

  std::shared_ptr<AbstractInput<T>> getInput() const {
    return input;
  }

protected:
  std::shared_ptr<AbstractInput<T>> input{nullptr};
  std::unique_ptr<okapi::Filter> filter{nullptr};

  T out{0};
};

  extern template class InputFilter<double>;


}
