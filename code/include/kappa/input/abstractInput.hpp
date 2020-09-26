#pragma once

namespace kappa {

template <typename T> class AbstractInput {
public:
  virtual const T &get() = 0;
};

}
