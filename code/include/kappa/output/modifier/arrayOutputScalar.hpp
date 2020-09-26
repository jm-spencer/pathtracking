#pragma once

#include "kappa/output/abstractOutput.hpp"
#include <array>
#include <memory>


namespace kappa {


template <typename T, std::size_t N>
class ArrayOutputScalar : public AbstractOutput<std::array<T,N>> {
public:
  ArrayOutputScalar(double iscalar, std::shared_ptr<AbstractOutput<std::array<T,N>>> ioutput):
    output(ioutput), scalar(iscalar) {}

  virtual void set(const std::array<T,N> &itarget) override {
    for(std::size_t i = 0; i < N; i++){
      target[i] = scalar * itarget[i];
    }
  }

  std::shared_ptr<AbstractOutput<std::array<T,N>>> getOutput() const {
    return output;
  }

protected:
  std::shared_ptr<AbstractOutput<std::array<T,N>>> output{nullptr};
  std::array<T,N> target;
  double scalar;

  static std::array<T,N> &scaleArray(const std::array<T,N> &arr, std::array<T,N> &target, T scalar) {
    for(std::size_t i = 0; i < N; i++){
      target[i] = arr[i] * scalar;
    }

    return target;
  }
};

extern template class ArrayOutputScalar<double, 2>;
extern template class ArrayOutputScalar<double, 4>;

}
