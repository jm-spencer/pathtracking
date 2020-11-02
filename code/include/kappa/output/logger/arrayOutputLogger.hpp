#pragma once

#include "kappa/output/abstractOutput.hpp"
#include "kappa/output/impl/nullOutput.hpp"
#include "pros/rtos.hpp"
#include <memory>
#include <array>
#include <ostream>
#include <iostream>
#include <iomanip>
#include <string>

namespace kappa {

template <typename T, std::size_t N>
class ArrayOutputLogger : public AbstractOutput<std::array<T,N>> {
public:

  /**
   * Logs array data that passes through it. By default logs to std::cout,
   * but can also log to filestreams
   * Assumes operator<<(std::ostream,T) is defined
   *
   * @param ioutput output for data
   */
  ArrayOutputLogger(std::shared_ptr<AbstractOutput<std::array<T,N>>> ioutput = std::make_shared<NullOutput<std::array<T,N>>>()):
    ArrayOutputLogger(" ", " ", "\n", std::cout, ioutput) {}

  /**
   * @param iprefix string that preceeds each line of data
   * @param iseparator string that is printed between each element of data
   * @param ipostfix string that follows each line of data (like a newline char)
   * @param ioutput output for data
   */
  ArrayOutputLogger(std::string iprefix, std::string iseparator, std::string ipostfix, std::shared_ptr<AbstractOutput<std::array<T,N>>> ioutput = std::make_shared<NullOutput<std::array<T,N>>>()):
    ArrayOutputLogger(iprefix, iseparator, ipostfix, std::cout, ioutput) {}

  /**
   * @param iprefix string that preceeds each line of data
   * @param iseparator string that is printed between each element of data
   * @param ipostfix string that follows each line of data (like a newline char)
   * @param iout ostream to print data to
   * @param ioutput output for data
   */
  ArrayOutputLogger(std::string iprefix, std::string iseparator, std::string ipostfix, std::ostream &iout, std::shared_ptr<AbstractOutput<std::array<T,N>>> ioutput = std::make_shared<NullOutput<std::array<T,N>>>()):
    output(ioutput), prefix(iprefix), separator(iseparator), postfix(ipostfix), out(iout) {}

  /**
   * Logs the target data and passes it to the output
   *
   * @param itarget target data
   */
  virtual void set(const std::array<T,N> &itarget) override {
    out << pros::millis() << prefix << itarget[0];

    for(std::size_t i = 1; i < N; i++){
      out << separator << itarget[i];
    }

    out << postfix;

    output->set(itarget);
  }

  /**
   * Gets output
   *
   * @return output
   */
  std::shared_ptr<AbstractOutput<std::array<T,N>>> getOutput() const {
    return output;
  };

protected:
  std::shared_ptr<AbstractOutput<std::array<T,N>>> output{nullptr};

  std::string prefix;
  std::string separator;
  std::string postfix;

  std::ostream &out;
};

extern template class ArrayOutputLogger<double, 2>;
extern template class ArrayOutputLogger<double, 3>;
extern template class ArrayOutputLogger<double, 4>;

}
