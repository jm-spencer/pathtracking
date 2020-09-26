#pragma once

#include "kappa/output/abstractOutput.hpp"


namespace kappa {

/*
 * Does nothing. If you want to test a controller without motors (or are running this on a computer)
 * Use a logger piping to this. Loggers will assume NullOutput if no output is specified.
 */

template<typename T>
class NullOutput : public AbstractOutput<T> {
public:
  NullOutput() = default;

  virtual void set(const T &itarget) override { }

};

extern template class NullOutput<double>;

}
