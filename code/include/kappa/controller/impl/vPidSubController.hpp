#pragma once

#include "kappa/controller/abstractSubController.hpp"
#include "okapi/api/control/util/settledUtil.hpp"
#include "okapi/api/filter/filter.hpp"
#include "okapi/impl/util/timeUtilFactory.hpp"

namespace kappa {

class VPidSubController : public AbstractSubController<double, double, double> {
public:

  struct Gains {
    double kP;
    double kD;
    double kF;
    double kSF;
  };

  VPidSubController(Gains igains,
                    std::shared_ptr<AbstractInput<double>> iinput,
                    std::shared_ptr<AbstractOutput<double>> ioutput);

  VPidSubController(Gains igains,
                    std::unique_ptr<okapi::Filter> iderivativeFilter,
                    std::shared_ptr<AbstractInput<double>> iinput,
                    std::shared_ptr<AbstractOutput<double>> ioutput);

  VPidSubController(Gains igains,
                    double ioutputMin,
                    double ioutputMax,
                    std::shared_ptr<AbstractInput<double>> iinput,
                    std::shared_ptr<AbstractOutput<double>> ioutput);

  VPidSubController(Gains igains,
                    double ioutputMin,
                    double ioutputMax,
                    std::unique_ptr<okapi::Filter> iderivativeFilter,
                    std::shared_ptr<AbstractInput<double>> iinput,
                    std::shared_ptr<AbstractOutput<double>> ioutput);

  virtual void set(const double &itarget) override;

  virtual void reset() override;

protected:
  Gains gains;
  std::unique_ptr<okapi::Filter> derivativeFilter{nullptr};

  double currentVelocity{0};

  double lastError{0};
  double derivative{0};

  double closedLoopSignal{0};
};

}
