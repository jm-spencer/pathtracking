#pragma once

#include "kappa/output/abstractOutput.hpp"
#include "kappa/output/impl/nullOutput.hpp"
#include "display/lv_objx/lv_gauge.h"
#include <memory>

namespace kappa {

// Note: T must be able to cast to int16

template<typename T>
class OutputGaugeLogger : public AbstractOutput<T> {
public:
  OutputGaugeLogger(lv_obj_t *igauge, uint8_t needle_id, std::shared_ptr<AbstractOutput<T>> ioutput = std::make_shared<NullOutput<T>>()):
    output(ioutput), gauge(igauge), id(needle_id) {}

  virtual void set(const T &itarget) override {
    lv_gauge_set_value(gauge, id, static_cast<int16_t>(itarget));
    output->set(itarget);
  }

  std::shared_ptr<AbstractOutput<T>> getOutput() const {
    return output;
  }

protected:
  std::shared_ptr<AbstractOutput<T>> output;
  lv_obj_t *gauge;
  uint8_t id;
};

extern template class OutputGaugeLogger<double>;

}
