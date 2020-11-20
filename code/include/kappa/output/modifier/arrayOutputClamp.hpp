#pragma once

#include "kappa/output/abstractOutput.hpp"
#include <array>
#include <algorithm>
#include <memory>


namespace kappa {

template <typename T, std::size_t N>
class ArrayOutputClamp : public AbstractOutput<std::array<T,N>> {
public:

  /**
   * Constrains control signals within a given domain, while preserving ratios therebetween
   * Requires definition of <, >, /, and *= for type T
   *
   * @param imin minimum value
   * @param imax maximum value
   * @param ioutput output for data
   */
  ArrayOutputClamp(T imin, T imax, std::shared_ptr<AbstractOutput<std::array<T,N>>> ioutput):
    output(ioutput), min(imin), max(imax) {}

  /**
   * Calculates the constrained target signals and sets them to the output
   *
   * @param itarget targets
   */
  virtual void set(const std::array<T,N> &itarget) override {
    T maxVal = *std::max_element(itarget.begin(), itarget.end());
    T minVal = *std::min_element(itarget.begin(), itarget.end());

    if(max < maxVal) {
      if(min > minVal) {
        output->set(scaleArray(itarget, target, (max / maxVal) < (min / minVal) ? (max / maxVal) : (min / minVal)));
      } else {
        output->set(scaleArray(itarget, target, max / maxVal));
      }
    } else {
      if(min > minVal) {
        output->set(scaleArray(itarget, target, min / minVal));
      } else {
        output->set(itarget);
      }
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
  T min;
  T max;

  static std::array<T,N> &scaleArray(const std::array<T,N> &arr, std::array<T,N> &target, T scalar) {
    for(std::size_t i = 0; i < N; i++){
      target[i] = arr[i] * scalar;
    }

    return target;
  }
};

extern template class ArrayOutputClamp<double, 2>;
extern template class ArrayOutputClamp<double, 4>;

}
