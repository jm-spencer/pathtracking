#pragma once

#include "kappa/output/abstractOutput.hpp"
#include "kappa/output/impl/nullOutput.hpp"
#include "display/lv_objx/lv_chart.h"
#include <memory>

namespace kappa {

template<typename T>
class OutputChartLogger : public AbstractOutput<T> {
public:

  /**
   * Logs data that passes through it via the lvgl chart object
   * Assumes T can cast to int16_t
   *
   * @param ichart lvgl chart object
   * @param iser lvgl chart series object
   * @param ioutput output for data
   */
  OutputChartLogger(lv_obj_t *ichart, lv_chart_series_t *iser, std::shared_ptr<AbstractOutput<T>> ioutput = std::make_shared<NullOutput<T>>()):
    output(ioutput), chart(ichart), ser(iser) {}

  /**
   * Logs the target data and passes it to the output
   *
   * @param itarget target data
   */
  virtual void set(const T &itarget) override {
    lv_chart_set_next(chart, ser, static_cast<int16_t>(itarget));
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
  lv_obj_t *chart;
  lv_chart_series_t *ser;
};

extern template class OutputChartLogger<double>;

}
