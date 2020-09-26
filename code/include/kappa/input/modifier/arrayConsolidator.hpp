#pragma once

#include "kappa/input/abstractInput.hpp"
#include <initializer_list>
#include <array>
#include <memory>


namespace kappa {

template<typename T, std::size_t N>
class ArrayConsolidator : public AbstractInput<std::array<T,N>> {
public:
  ArrayConsolidator(std::initializer_list<std::shared_ptr<AbstractInput<T>>> iinput) {
    if (iinput.size() != N) throw std::invalid_argument("Invalid number of args in kappa::ArrayConsolidator");

    std::copy(iinput.begin(), iinput.end(), input.begin());
  }


  virtual const std::array<T,N> &get() override {
    for(std::size_t i = 0; i < N; i++) {
      out[i] = input[i]->get();
    }
    return out;
  }

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
