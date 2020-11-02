#pragma once

#include "kappa/output/abstractOutput.hpp"
#include "kappa/output/impl/nullOutput.hpp"
#include "pros/rtos.hpp"
#include <memory>
#include <ostream>
#include <iostream>
#include <iomanip>
#include <string>

namespace kappa {

template <typename T>
class OutputLogger : public AbstractOutput<T> {
public:

  /**
   * Logs data that passes through it. By default logs to std::cout,
   * but can also log to filestreams
   * Assumes operator<<(std::ostream,T) is defined
   *
   * @param ioutput output for data
   */
  OutputLogger(std::shared_ptr<AbstractOutput<T>> ioutput = std::make_shared<NullOutput<T>>()):
    OutputLogger(" ", "\n", std::cout, ioutput) {}

  /**
   * @param iprefix string that preceeds each line of data
   * @param ipostfix string that follows each line of data (like a newline char)
   * @param ioutput output for data
   */
  OutputLogger(std::string iprefix, std::string ipostfix, std::shared_ptr<AbstractOutput<T>> ioutput = std::make_shared<NullOutput<T>>()):
    OutputLogger(iprefix, ipostfix, std::cout, ioutput) {}

  /**
   * @param iprefix string that preceeds each line of data
   * @param ipostfix string that follows each line of data (like a newline char)
   * @param iout ostream to print data to
   * @param ioutput output for data
   */
  OutputLogger(std::string iprefix, std::string ipostfix, std::ostream &iout, std::shared_ptr<AbstractOutput<T>> ioutput = std::make_shared<NullOutput<T>>()):
    output(ioutput), prefix(iprefix), postfix(ipostfix), out(iout) {}

  /**
   * Logs the target data and passes it to the output
   *
   * @param itarget target data
   */
  virtual void set(const T &itarget) override {
    out << pros::millis() << prefix << itarget << postfix;
    output->set(itarget);
  }

  /**
   * Gets output
   *
   * @return output
   */
  std::shared_ptr<AbstractOutput<T>> getOutput() const {
    return output;
  };

protected:
  std::shared_ptr<AbstractOutput<T>> output{nullptr};

  std::string prefix;
  std::string postfix;

  std::ostream &out;
};

extern template class OutputLogger<double>;

}
