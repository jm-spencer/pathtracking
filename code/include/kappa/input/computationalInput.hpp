#pragma once

#include "kappa/input/abstractInput.hpp"

namespace kappa {

/*
 * In this implimentation, get() will NOT update internal values, 
 * while step() will update internal values.
 * It is suggested to allocate a seperate task to calling step()
 * at a specified interval
 */

template <typename T>
class ComputationalInput : public AbstractInput<T> {
public:
  virtual const T &step() = 0;
};

}
