#pragma once

#include "kappa/output/abstractOutput.hpp"
#include <algorithm>
#include <memory>


namespace kappa {

template <typename T>
class OutputClamp : public AbstractOutput<T> {
public:

  /**
   * Constrains control signal within a given domain
   * Requires definition of < for type T
   *
   * @param imin minimum value
   * @param imax maximum value
   * @param ioutput output for constrained data
   */
  OutputClamp(T imin, T imax, std::shared_ptr<AbstractOutput<T>> ioutput):
    output(ioutput), min(imin), max(imax) {}

  /**
   * Calculates the constrained target signal and sets it to the output
   *
   * @param itarget target
   */
  virtual void set(const T &itarget) override {
    output->set(std::clamp(itarget, min, max));
  }

  /**
   * Gets output
   *
   * @return output
   */
  std::shared_ptr<AbstractOutput<T>> getOutput() const {
    return output;
  }

protected:
  std::shared_ptr<AbstractOutput<T>> output{nullptr};
  T min{0};
  T max{0};
};

extern template class OutputClamp<double>;

}
