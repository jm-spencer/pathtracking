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
  InputLogger(std::shared_ptr<AbstractInput<T>> iinput):
    InputLogger(6, " ", "\n", std::cout, iinput) {}

  InputLogger(int iprecision, std::string iprefix, std::string ipostfix, std::shared_ptr<AbstractInput<T>> iinput):
    InputLogger(iprecision, iprefix, ipostfix, std::cout, iinput) {}

  InputLogger(int iprecision, std::string iprefix, std::string ipostfix, std::ostream &iout, std::shared_ptr<AbstractInput<T>> iinput):
    input(iinput), prefix(iprefix), postfix(ipostfix), out(iout) {

    out << std::setprecision(iprecision);
  }

  virtual const T &get() override {
    const T &value = input->get();
    out << pros::millis() << prefix << value << postfix;
    return value;
  }

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
