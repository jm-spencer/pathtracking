#pragma once

#include "kappa/input/abstractInput.hpp"
#include <initializer_list>
#include <array>
#include <memory>


namespace kappa {

template<typename T, std::size_t N>
class ArrayConsolidator : public AbstractInput<std::array<T,N>> {
public:

  /**
   * Packages the data from an array of inputs into a single array of data
   *
   * @param iinput inputs for data
   */
  ArrayConsolidator(std::initializer_list<std::shared_ptr<AbstractInput<T>>> iinput) {
    if (iinput.size() != N) throw std::invalid_argument("Invalid number of args in kappa::ArrayConsolidator");

    std::copy(iinput.begin(), iinput.end(), input.begin());
  }

  /**
   * Gets data from its inputs, and returns array of data
   *
   * @return input data
   */
  virtual const std::array<T,N> &get() override {
    for(std::size_t i = 0; i < N; i++) {
      out[i] = input[i]->get();
    }
    return out;
  }

  /**
   * Gets input sources
   *
   * @return inputs
   */
  std::array<std::shared_ptr<AbstractInput<T>>,N> getInput() const {
    return input;
  };

protected:
  std::array<std::shared_ptr<AbstractInput<T>>,N> input;
  std::array<T,N> out;
};

extern template class ArrayConsolidator<double, 2>;
extern template class ArrayConsolidator<double, 3>;
extern template class ArrayConsolidator<double, 4>;

}
