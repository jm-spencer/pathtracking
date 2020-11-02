#pragma once

#include "kappa/input/abstractInput.hpp"
#include "kappa/util/tupleLogger.hpp"
#include "pros/rtos.hpp"
#include <memory>
#include <tuple>
#include <ostream>
#include <iostream>
#include <iomanip>
#include <string>

namespace kappa {

template <typename...T>
class TupleInputLogger : public AbstractInput<std::tuple<T...>> {
public:

  /**
   * Logs tuple data that passes through it. By default logs to std::cout,
   * but can also log to filestreams
   * Assumes operator<<(std::ostream,T) is defined
   *
   * @param iinput input for data
   */
  TupleInputLogger(std::shared_ptr<AbstractInput<std::tuple<T...>>> iinput):
    TupleInputLogger(", ", ", ", "\n", std::cout, iinput) {}

  /**
   * @param iprefix string that preceeds each line of data
   * @param iseparator string that is printed between each element of data
   * @param ipostfix string that follows each line of data (like a newline char)
   * @param iinput input for data
   */
  TupleInputLogger(std::string iprefix, std::string iseparator, std::string ipostfix, std::shared_ptr<AbstractInput<std::tuple<T...>>> iinput):
    TupleInputLogger(prefix, iseparator, ipostfix, std::cout, iinput) {}

  /**
   * @param iprefix string that preceeds each line of data
   * @param iseparator string that is printed between each element of data
   * @param ipostfix string that follows each line of data (like a newline char)
   * @param iout ostream to print data to
   * @param iinput input for data
   */
  TupleInputLogger(std::string iprefix, std::string iseparator, std::string ipostfix, std::ostream &iout, std::shared_ptr<AbstractInput<std::tuple<T...>>> iinput):
    input(iinput), prefix(iprefix), separator(iseparator), postfix(ipostfix), out(iout) {}

  /**
   * Gets data from its input, logs it, and returns the data
   *
   * @return input data
   */
  virtual const std::tuple<T...> &get() override {
    const std::tuple<T...> &values = input->get();

    out << pros::millis() << prefix;

    printTuple(values, out, separator);

    out << postfix;

    return values;
  }

  /**
   * Gets input source
   *
   * @return input
   */
  std::shared_ptr<AbstractInput<std::tuple<T...>>> getInput() const {
    return input;
  }

protected:
  std::shared_ptr<AbstractInput<std::tuple<T...>>> input{nullptr};

  std::string prefix;
  std::string separator;
  std::string postfix;

  std::ostream &out;
};

}
