#pragma once

#include "kappa/input/abstractInput.hpp"
#include "pros/rtos.hpp"
#include <memory>
#include <array>
#include <ostream>
#include <iostream>
#include <iomanip>
#include <string>

namespace kappa {

template <typename T, std::size_t N>
class ArrayInputLogger : public AbstractInput<std::array<T,N>> {
public:

  /**
   * Logs array data that passes through it. By default logs to std::cout,
   * but can also log to filestreams
   * Assumes operator<<(std::ostream,T) is defined
   *
   * @param iinput input for data
   */
  ArrayInputLogger(std::shared_ptr<AbstractInput<std::array<T,N>>> iinput):
    ArrayInputLogger(", ", ", ", "\n", std::cout, iinput) {}

  /**
   * @param iprefix string that preceeds each line of data
   * @param iseparator string that is printed between each element of data
   * @param ipostfix string that follows each line of data (like a newline char)
   * @param iinput input for data
   */
  ArrayInputLogger(std::string iprefix, std::string iseparator, std::string ipostfix, std::shared_ptr<AbstractInput<std::array<T,N>>> iinput):
    ArrayInputLogger(iprefix, iseparator, ipostfix, std::cout, iinput) {}

  /**
   * @param iprefix string that preceeds each line of data
   * @param iseparator string that is printed between each element of data
   * @param ipostfix string that follows each line of data (like a newline char)
   * @param iout ostream to print data to
   * @param iinput input for data
   */
  ArrayInputLogger(std::string iprefix, std::string iseparator, std::string ipostfix, std::ostream &iout, std::shared_ptr<AbstractInput<std::array<T,N>>> iinput):
    input(iinput), prefix(iprefix), separator(iseparator), postfix(ipostfix), out(iout) {}

  /**
   * Gets data from its input, logs it, and returns the data
   *
   * @return input data
   */
  virtual const std::array<T,N> &get() override {
    const std::array<T,N> &values = input->get();

    out << pros::millis() << prefix << values[0];

    for(std::size_t i = 1; i < N; i++){
      out << separator << values[i];
    }

    out << postfix;

    return values;
  }

  /**
   * Gets input source
   *
   * @return input
   */
  std::shared_ptr<AbstractInput<std::array<T,N>>> getInput() const {
    return input;
  }

protected:
  std::shared_ptr<AbstractInput<std::array<T,N>>> input{nullptr};

  std::string prefix;
  std::string separator;
  std::string postfix;

  std::ostream &out;
};

extern template class ArrayInputLogger<double, 2>;
extern template class ArrayInputLogger<double, 3>;
extern template class ArrayInputLogger<double, 4>;

}
