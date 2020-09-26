#pragma once

#include "kappa/input/abstractInput.hpp"
#include "display/lv_objx/lv_chart.h"
#include <memory>

namespace kappa {

// Note: T must be able to cast to int16

template<typename T>
class InputChartLogger : public AbstractInput<T> {
public:
  InputChartLogger(lv_obj_t *ichart, lv_chart_series_t *iser, std::shared_ptr<AbstractInput<T>> iinput):
    input(iinput), chart(ichart), ser(iser) {}

  virtual const T &get() override {
    const T &value = input->get();
    lv_chart_set_next(chart, ser, static_cast<int16_t>(value));
    return value;
  }

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
