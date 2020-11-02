#pragma once

#include "kappa/output/abstractOutput.hpp"
#include <array>
#include <memory>


namespace kappa {

template <typename T, std::size_t N>
class ArrayOutputScalar : public AbstractOutput<std::array<T,N>> {
public:

  /**
   * Scales control signals by a preset scalar
   * Requires definition of operator*(double,T)
   *
   * @param iscalar scalar
   * @param ioutput output for data
   */
  ArrayOutputScalar(double iscalar, std::shared_ptr<AbstractOutput<std::array<T,N>>> ioutput):
    output(ioutput), scalar(iscalar) {}

  /**
   * Calculates the scaled target signals and sets them to the output
   *
   * @param itarget targets
   */
  virtual void set(const std::array<T,N> &itarget) override {
    for(std::size_t i = 0; i < N; i++){
      target[i] = scalar * itarget[i];
    }
  }

  /**
   * Gets output
   *
   * @return output
   */
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
