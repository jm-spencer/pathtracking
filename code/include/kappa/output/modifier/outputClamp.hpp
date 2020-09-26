#pragma once

#include "kappa/output/abstractOutput.hpp"
#include <algorithm>
#include <memory>


namespace kappa {

template <typename T>
class OutputClamp : public AbstractOutput<T> {
public:
  OutputClamp(T imin, T imax, std::shared_ptr<AbstractOutput<T>> ioutput):
    output(ioutput), min(imin), max(imax) {}

  virtual void set(const T &itarget) override {
    output->set(std::clamp(itarget, min, max));
  }

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
