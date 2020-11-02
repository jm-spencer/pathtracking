#pragma once

#include "kappa/input/abstractInput.hpp"
#include "pros/rtos.hpp"
#include <memory>
#include <ostream>
#include <iostream>
#include <iomanip>
#include <string>

namespace kappa {

template <typename T>
class InputLogger : public AbstractInput<T> {
public:

  /**
   * Logs data that passes through it. By default logs to std::cout,
   * but can also log to filestreams.
   * Assumes operator<<(std::ostream,T) is defined
   *
   * @param iinput input for data
   */
  InputLogger(std::shared_ptr<AbstractInput<T>> iinput):
    InputLogger(" ", "\n", std::cout, iinput) {}

  /**
   * @param iprefix string that preceeds each line of data
   * @param ipostfix string that follows each line of data (like a newline char)
   * @param iinput input for data
   */
  InputLogger(std::string iprefix, std::string ipostfix, std::shared_ptr<AbstractInput<T>> iinput):
    InputLogger(iprefix, ipostfix, std::cout, iinput) {}

  /**
   * @param iprefix string that preceeds each line of data
   * @param ipostfix string that follows each line of data (like a newline char)
   * @param iout ostream to print data to
   * @param iinput input for data
   */
  InputLogger(std::string iprefix, std::string ipostfix, std::ostream &iout, std::shared_ptr<AbstractInput<T>> iinput):
    input(iinput), prefix(iprefix), postfix(ipostfix), out(iout) {}

  /**
   * Gets data from its input, logs it, and returns the data
   *
   * @return input data
   */
  virtual const T &get() override {
    const T &value = input->get();
    out << pros::millis() << prefix << value << postfix;
    return value;
  }

  /**
   * Gets input source
   *
   * @return input
   */
  std::shared_ptr<AbstractInput<T>> getInput() const {
    return input;
  }

protected:
  std::shared_ptr<AbstractInput<T>> input{nullptr};

  std::string prefix;
  std::string postfix;

  std::ostream &out;
};

extern template class InputLogger<double>;

}
