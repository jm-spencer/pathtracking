#pragma once

#include "kappa/output/abstractOutput.hpp"
#include "kappa/output/impl/nullOutput.hpp"
#include "kappa/util/tupleLogger.hpp"
#include "pros/rtos.hpp"
#include <memory>
#include <tuple>
#include <ostream>
#include <iostream>
#include <iomanip>
#include <string>

namespace kappa {

template <typename... T>
class TupleOutputLogger : public AbstractOutput<std::tuple<T...>> {
public:

  /**
   * Logs tuple data that passes through it. By default logs to std::cout,
   * but can also log to filestreams
   * Assumes operator<<(std::ostream,T) is defined
   *
   * @param ioutput output for data
   */
  TupleOutputLogger(std::shared_ptr<AbstractOutput<std::tuple<T...>>> ioutput = std::make_shared<NullOutput<std::tuple<T...>>>()):
    TupleOutputLogger(", ", ", ", "\n", std::cout, ioutput) {}

  /**
   * @param iprefix string that preceeds each line of data
   * @param iseparator string that is printed between each element of data
   * @param ipostfix string that follows each line of data (like a newline char)
   * @param ioutput output for data
   */
  TupleOutputLogger(std::string iprefix, std::string iseparator, std::string ipostfix, std::shared_ptr<AbstractOutput<std::tuple<T...>>> ioutput = std::make_shared<NullOutput<std::tuple<T...>>>()):
    TupleOutputLogger(iprefix, iseparator, ipostfix, std::cout, ioutput) {}

  /**
   * @param iprefix string that preceeds each line of data
   * @param iseparator string that is printed between each element of data
   * @param ipostfix string that follows each line of data (like a newline char)
   * @param iout ostream to print data to
   * @param ioutput output for data
   */
  TupleOutputLogger(std::string iprefix, std::string iseparator, std::string ipostfix, std::ostream &iout, std::shared_ptr<AbstractOutput<std::tuple<T...>>> ioutput = std::make_shared<NullOutput<std::tuple<T...>>>()):
    output(ioutput), prefix(iprefix), separator(iseparator), postfix(ipostfix), out(iout) {}

  /**
   * Logs the target data and passes it to the output
   *
   * @param itarget target data
   */
  virtual void set(const std::tuple<T...> &itarget) override {
    out << pros::millis() << prefix;

    printTuple(itarget, out, separator);

    out << postfix;

    output->set(itarget);
  }

  /**
   * Gets output
   *
   * @return output
   */
  std::shared_ptr<AbstractOutput<std::tuple<T...>>> getOutput() const {
    return output;
  };

protected:
  std::shared_ptr<AbstractOutput<std::tuple<T...>>> output{nullptr};

  std::string prefix;
  std::string separator;
  std::string postfix;

  std::ostream &out;
};

extern template class TupleOutputLogger<double,double>;
extern template class TupleOutputLogger<double,double,double>;

}
