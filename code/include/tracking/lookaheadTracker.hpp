#pragma once
#include "kappa/api.hpp"
#include "tracking/abstractTracker.hpp"

template <std::size_t N>
class LookaheadTracker: public AbstractTracker<N> {
public:
  LookaheadTracker(double ilookaheadDist):
    lookaheadDist(ilookaheadDist),
    lookaheadDistSqr(ilookaheadDist * ilookaheadDist){

    lastWaypoint.fill(0);
  }

  std::array<double,N> getGoalPoint(double robotx, double roboty) {
    return getGoalPoint(robotx, roboty, lookaheadDistSqr);
  }

  std::array<double,N> getGoalPoint(double robotx, double roboty, double effectiveLookaheadSqr);

  std::array<double,N> globalToLocalCoords(const std::array<double,N> &point, const std::array<double,6> &basis);

  void skipPoint(uint recurse){
    if(!recurse) return;
    
    std::copy(AbstractTracker<N>::activeWaypoint.begin(), AbstractTracker<N>::activeWaypoint.end(), lastWaypoint.begin());
    AbstractTracker<N>::activeWaypoint = AbstractTracker<N>::pathFile->get();
    waypointIndex++;
    skipPoint(recurse - 1);
  }

protected:
  double lookaheadDist;
  double lookaheadDistSqr;

  double waypointIndex{0};

  std::array<double,N> lastWaypoint;
};
