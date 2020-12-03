#pragma once
#include "kappa/api.hpp"
#include "tracking/lookaheadTracker.hpp"

class PurePursuitAdaptive1Tracker: public kappa::AbstractController<std::array<double,6>,uint,std::tuple<double,double>> {
public:
  PurePursuitAdaptive1Tracker(double ispeedTarget, double ilookaheadDist);

  virtual void setTarget(const uint &itarget) override;

  virtual std::tuple<double,double> step(std::array<double,6> ireading) override;

  virtual bool isSettled() override;

  virtual void reset() override;

  virtual void disable(bool iisDisabled) override;

  void skipPoint(uint recurse);

protected:
  std::array<double,2> getGoalPoint(double robotx, double roboty, double adaptLookaheadDistSqr);
  double getLateralError(double robotx, double roboty);

  std::array<double,2> globalToLocalCoords(const std::array<double,2> &point, const std::array<double,6> &basis);

  double speedTarget{0};
  double lookaheadDist{0};
  double lookaheadDistSqr{0};

  std::array<double,2> lastLookaheadWaypoint{0,0};
  std::array<double,2> lastNearestWaypoint{0,0};
  std::array<double,2> activeLookaheadWaypoint{0,0};
  std::array<double,2> activeNearestWaypoint{0,0};

  std::shared_ptr<kappa::AbstractInput<std::array<double,2>>> pathFileL{nullptr};
  std::shared_ptr<kappa::AbstractInput<std::array<double,2>>> pathFileN{nullptr};

  bool finished{false};
};
