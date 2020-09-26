#pragma once

#include "kappa/input/abstractInput.hpp"
#include "display/lv_objx/lv_gauge.h"
#include <memory>

namespace kappa {

// Note: T must be able to cast to int16

template<typename T>
class InputGaugeLogger : public AbstractInput<T> {
public:
  InputGaugeLogger(lv_obj_t *igauge, uint8_t needle_id, std::shared_ptr<AbstractInput<T>> iinput):
    input(iinput), gauge(igauge), id(needle_id) {}

  virtual const T &get() override {
    const T &value = input->get();
    lv_gauge_set_value(gauge, id, static_cast<int16_t>(value));
    return value;
  }

  std::shared_ptr<AbstractInput<T>> getInput() const {
    return input;
  }

protected:
  std::shared_ptr<AbstractInput<T>> input;
  lv_obj_t *gauge;
  uint8_t id;
};

extern template class InputGaugeLogger<double>;

}
