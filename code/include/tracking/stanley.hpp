#pragma once
#include "kappa/api.hpp"
#include "tracking/lateralTracker.hpp"

class StanleyTracker : public LateralTracker<3> {
public:
  StanleyTracker(double ik, double il, double ispeedTarget);

  virtual std::tuple<double,double> step(std::array<double,6> ireading) override;

  virtual bool isSettled() override;

  virtual void reset() override;

  virtual void disable(bool iisDisabled) override;

protected:
  double speedTarget{0};
  double k{0};
  double l{0};
};
