#pragma once

#include "kappa/output/abstractOutput.hpp"
#include "kappa/output/impl/nullOutput.hpp"
#include "display/lv_objx/lv_gauge.h"
#include <memory>

namespace kappa {

template<typename T>
class OutputGaugeLogger : public AbstractOutput<T> {
public:

  /**
   * Logs data that passes through it via the lvgl gauge object
   * Assumes T can cast to int16_t
   *
   * @param igauge lvgl gauge object
   * @param needle_id needle id for gauge
   * @param ioutput output for data
   */
  OutputGaugeLogger(lv_obj_t *igauge, uint8_t needle_id, std::shared_ptr<AbstractOutput<T>> ioutput = std::make_shared<NullOutput<T>>()):
    output(ioutput), gauge(igauge), id(needle_id) {}

  /**
   * Logs the target data and passes it to the output
   *
   * @param itarget target data
   */
  virtual void set(const T &itarget) override {
    lv_gauge_set_value(gauge, id, static_cast<int16_t>(itarget));
    output->set(itarget);
  }

  /**
   * Gets output
   *
   * @return output
   */
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
