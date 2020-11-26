#pragma once
#include "kappa/api.hpp"
#include "tracking/lookaheadTracker.hpp"

class RamseteTracker : public LookaheadTracker<6> {
public:
  RamseteTracker(double izeta, double ib, double ispeedTarget, double ilookaheadDist);

  virtual std::tuple<double,double> step(std::array<double,6> ireading) override;

  virtual bool isSettled() override;

  virtual void reset() override;

  virtual void disable(bool iisDisabled) override;

protected:
  double speedTarget{0};
  double zeta{0};
  double b{0};
};
