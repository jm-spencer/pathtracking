#pragma once

#include "kappa/input/abstractInput.hpp"
#include "okapi/api/filter/filter.hpp"
#include "okapi/api/filter/passthroughFilter.hpp"
#include <memory>

namespace kappa {

template <typename T>
class InputFilter : public AbstractInput<T> {
public:

  /**
   * Filters data using an okapi filter
   * Assumes T can cast to double
   *
   * @param ifilter filter after differentiation
   * @param iinput input for data
   */
  InputFilter(std::unique_ptr<okapi::Filter> ifilter, std::shared_ptr<AbstractInput<T>> iinput):
    input(iinput), filter(std::move(ifilter)) {}

  /**
   * Gets data from its input, filters, and returns it
   *
   * @return input data
   */
  virtual const T &get() override {
    out = static_cast<T>(filter->filter(static_cast<double>(input->get())));
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

  T out;
};

  extern template class InputFilter<double>;


}
