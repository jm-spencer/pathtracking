#pragma once

#include "kappa/input/simpleInput.hpp"
#include <functional>

namespace kappa {

template <typename T>
class FunctionInput : public SimpleInput<T> {
public:
  
  /**
   * runs a function and passes forward its return value
   * 
   * @param ifn function to be run
   */
  FunctionInput(std::function<T()> ifn):
    fn(ifn){}

  /**
   * gets value from the function
   * 
   * @return value
   */
  virtual const T &get() override {
    value = fn();
    return value;
  }

  /**
   * gets previous value, without runing the function
   * 
   * @returns value
   */
  virtual const T &getValue() const override {
    return value;
  }

protected:
  std::function<T()> fn;
  T value;
};

}
