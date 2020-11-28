#pragma once
#include "kappa/api.hpp"
#include "tracking/lookaheadTracker.hpp"

class PurePursuitAdaptive2Tracker: public kappa::AbstractController<std::array<double,6>,uint,std::tuple<double,double>> {
public:
  PurePursuitAdaptive2Tracker(uint iN, double ispeedTarget, double ilookaheadDist);

  virtual void setTarget(const uint &itarget) override;

  virtual std::tuple<double,double> step(std::array<double,6> ireading) override;

  virtual bool isSettled() override;

  virtual void reset() override;

  virtual void disable(bool iisDisabled) override;

protected:
  std::array<double,2> getGoalPoint(double robotx, double roboty, double adaptLookaheadDistSqr);
  double getCurvature(double robotx, double roboty);

  std::array<double,2> globalToLocalCoords(const std::array<double,2> &point, const std::array<double,6> &basis);

  uint n{0};
  double speedTarget{0};
  double lookaheadDist{0};
  double lookaheadDistSqr{0};

  std::array<double,2> lastLookaheadWaypoint{0,0};
  std::array<double,3> lastNearestWaypoint{0,0,0};
  std::array<double,2> activeLookaheadWaypoint{0,0};
  std::array<double,3> activeNearestWaypoint{0,0,0};
  std::array<double,3> activeCurvatureWaypoint{0,0,0};

  std::shared_ptr<kappa::AbstractInput<std::array<double,2>>> pathFileL{nullptr};
  std::shared_ptr<kappa::AbstractInput<std::array<double,3>>> pathFileN{nullptr};
  std::shared_ptr<kappa::AbstractInput<std::array<double,3>>> pathFileC{nullptr};

  bool finished{false};
};
