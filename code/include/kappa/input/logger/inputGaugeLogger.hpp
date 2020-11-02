#pragma once

#include "kappa/input/abstractInput.hpp"
#include "display/lv_objx/lv_gauge.h"
#include <memory>

namespace kappa {

template<typename T>
class InputGaugeLogger : public AbstractInput<T> {
public:

  /**
   * Logs data that passes through it via the lvgl gauge object
   * Assumes T can cast to int16_t
   *
   * @param igauge lvgl gauge object
   * @param needle_id needle id for gauge
   * @param iinput input for data
   */
  InputGaugeLogger(lv_obj_t *igauge, uint8_t needle_id, std::shared_ptr<AbstractInput<T>> iinput):
    input(iinput), gauge(igauge), id(needle_id) {}

  /**
   * Gets data from its input, logs it, and returns the data
   *
   * @return input data
   */
  virtual const T &get() override {
    const T &value = input->get();
    lv_gauge_set_value(gauge, id, static_cast<int16_t>(value));
    return value;
  }

  /**
   * Gets input source
   *
   * @return input
   */
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
