#pragma once

#include "kappa/input/simpleInput.hpp"
#include <fstream>
#include <array>
#include <string>
#include <iostream>

namespace kappa {

template <typename T, std::size_t N>
class BinFileInput : public SimpleInput<std::array<T,N>> {
public:

  /**
   * Opens a binary file of T values. For each iteration, it takes N values
   * and passes it as an array of values
   *
   * @param filename path to file (make sure it begins with /usd/)
   */
  BinFileInput(const std::string &filename):
    file(filename, std::ios::binary){}

  ~BinFileInput() {
    file.close();
  }

  /**
   * gets values from the file
   *
   * @return values
   */
  virtual const std::array<T,N> &get() override {
    file.read((char*)value.data(), N * sizeof(T));
    return value;
  }

  /**
   * gets previous values, without getting new values from the file
   *
   * @returns values
   */
  virtual const std::array<T,N> &getValue() const override {
    return value;
  }

protected:
  std::ifstream file;
  std::array<T,N> value;
  bool finished{false};
};

extern template class BinFileInput<double,1>;
extern template class BinFileInput<double,2>;
extern template class BinFileInput<double,3>;
extern template class BinFileInput<double,4>;
extern template class BinFileInput<double,5>;
extern template class BinFileInput<double,6>;
extern template class BinFileInput<double,7>;

}
