#pragma once
#include "kappa/api.hpp"
#include "tracking/lookaheadTracker.hpp"

class VectorPursuitTracker: public LookaheadTracker<3> {
public:
  VectorPursuitTracker(double ik, double ispeedTarget, double ilookaheadDist);

  virtual std::tuple<double,double> step(std::array<double,6> ireading) override;

  virtual bool isSettled() override;

  virtual void reset() override;

  virtual void disable(bool iisDisabled) override;

protected:
  double speedTarget{0};
  double k{0};
};
