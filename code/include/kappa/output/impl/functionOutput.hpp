#pragma once

#include "kappa/output/abstractOutput.hpp"
#include <functional>


namespace kappa {

template<typename T>
class FunctionOutput : public AbstractOutput<T> {
public:

  /**
   * passes the target value to a function
   * 
   * @param ifn function to be run
   */
  FunctionOutput(std::function<void(const T&)> ifn):
    fn(ifn){}

  /**
   * runs the function with the target value
   * 
   * @param itarget target value
   */
  virtual void set(const T &itarget) override {
    fn(itarget);
  }

protected:
  std::function<void(const T&)> fn;
};

}
