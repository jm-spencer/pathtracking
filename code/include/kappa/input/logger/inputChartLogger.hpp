#pragma once

#include "kappa/input/abstractInput.hpp"
#include "display/lv_objx/lv_chart.h"
#include <memory>

namespace kappa {

template<typename T>
class InputChartLogger : public AbstractInput<T> {
public:

  /**
   * Logs data that passes through it via the lvgl chart object
   * Assumes T can cast to int16_t
   *
   * @param ichart lvgl chart object
   * @param iser lvgl chart series object
   * @param iinput input for data
   */
  InputChartLogger(lv_obj_t *ichart, lv_chart_series_t *iser, std::shared_ptr<AbstractInput<T>> iinput):
    input(iinput), chart(ichart), ser(iser) {}

  /**
   * Gets data from its input, logs it, and returns the data
   *
   * @return input data
   */
  virtual const T &get() override {
    const T &value = input->get();
    lv_chart_set_next(chart, ser, static_cast<int16_t>(value));
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
  lv_obj_t *chart;
  lv_chart_series_t *ser;
};

extern template class InputChartLogger<double>;

}
