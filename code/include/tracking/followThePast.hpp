#pragma once
#include "kappa/api.hpp"
#include "tracking/nearestTracker.hpp"

class FollowThePastTracker : public NearestTracker<4> {
public:
  FollowThePastTracker(double il, double ilookaheadDist, double ispeedTarget);

  virtual std::tuple<double,double> step(std::array<double,6> ireading) override;

  virtual bool isSettled() override;

  virtual void reset() override;

  virtual void disable(bool iisDisabled) override;

protected:
  double speedTarget{0};
  double lookaheadDist{0};
  double l{0};
};
