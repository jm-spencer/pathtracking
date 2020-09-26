#pragma once

#include "kappa/output/abstractOutput.hpp"
#include <memory>


namespace kappa {

template <typename T>
class OutputScalar : public AbstractOutput<T> {
public:
  OutputScalar(double iscalar, std::shared_ptr<AbstractOutput<T>> ioutput):
    output(ioutput), scalar(iscalar) {}

  virtual void set(const T &itarget) override {
    target = scalar * itarget;
    output->set(target);
  }

  std::shared_ptr<AbstractOutput<T>> getOutput() const {
    return output;
  }

protected:
  std::shared_ptr<AbstractOutput<T>> output{nullptr};
  T target{0};
  double scalar{0};
};

extern template class OutputScalar<double>;

}
