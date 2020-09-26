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
  OutputLogger(std::shared_ptr<AbstractOutput<T>> ioutput = std::make_shared<NullOutput<T>>()):
    OutputLogger(6, " ", "\n", std::cout, ioutput) {}

  OutputLogger(int iprecision, std::string iprefix, std::string ipostfix, std::shared_ptr<AbstractOutput<T>> ioutput = std::make_shared<NullOutput<T>>()):
    OutputLogger(iprecision, iprefix, ipostfix, std::cout, ioutput) {}

  OutputLogger(int iprecision, std::string iprefix, std::string ipostfix, std::ostream &iout, std::shared_ptr<AbstractOutput<T>> ioutput = std::make_shared<NullOutput<T>>()):
    output(ioutput), prefix(iprefix), postfix(ipostfix), out(iout) {

    out << std::setprecision(iprecision);
  }

  virtual void set(const T &itarget) override {
    out << pros::millis() << prefix << itarget << postfix;
    output->set(itarget);
  }

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
