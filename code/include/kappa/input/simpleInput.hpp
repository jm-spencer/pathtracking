#pragma once

#include "kappa/input/abstractInput.hpp"

namespace kappa {

/*
 * In this implimentation, get() WILL update internal values, 
 * while getValue() will not update internal values.
 */

template <typename T>
class SimpleInput : public AbstractInput<T> {
public:
  virtual const T &getValue() const = 0;
};

}
