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
  ArrayOutputLogger(std::shared_ptr<AbstractOutput<std::array<T,N>>> ioutput = std::make_shared<NullOutput<std::array<T,N>>>()):
    ArrayOutputLogger(6, " ", " ", "\n", std::cout, ioutput) {}

  ArrayOutputLogger(int iprecision, std::string iprefix, std::string iseparator, std::string ipostfix, std::shared_ptr<AbstractOutput<std::array<T,N>>> ioutput = std::make_shared<NullOutput<std::array<T,N>>>()):
    ArrayOutputLogger(iprecision, iprefix, iseparator, ipostfix, std::cout, ioutput) {}

  ArrayOutputLogger(int iprecision, std::string iprefix, std::string iseparator, std::string ipostfix, std::ostream &iout, std::shared_ptr<AbstractOutput<std::array<T,N>>> ioutput = std::make_shared<NullOutput<std::array<T,N>>>()):
    output(ioutput), prefix(iprefix), separator(iseparator), postfix(ipostfix), out(iout) {

    out << std::setprecision(iprecision);
  }

  virtual void set(const std::array<T,N> &itarget) override {
    out << pros::millis() << prefix << itarget[0];

    for(std::size_t i = 1; i < N; i++){
      out << separator << itarget[i];
    }

    out << postfix;

    output->set(itarget);
  }

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
