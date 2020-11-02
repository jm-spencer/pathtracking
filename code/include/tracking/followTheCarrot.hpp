#pragma once
#include "kappa/api.hpp"
#include "tracking/lookaheadTracker.hpp"

class FollowTheCarrotTracker: public LookaheadTracker<2> {
public:
  FollowTheCarrotTracker(double ikP, double ispeedTarget, double ilookaheadDist);

  virtual std::tuple<double,double> step(std::array<double,6> ireading) override;

  virtual bool isSettled() override;

  virtual void reset() override;

  virtual void disable(bool iisDisabled) override;

protected:
  double kP{0};
  double speedTarget{0};
};
