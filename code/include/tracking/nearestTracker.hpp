#pragma once
#include "kappa/api.hpp"
#include "tracking/abstractTracker.hpp"

template <std::size_t N>
class NearestTracker: public AbstractTracker<N> {
public:
  NearestTracker(){
    lastWaypoint.fill(0);
  }

  std::array<double,N> getGoalPoint(double robotx, double roboty);

protected:
  double waypointIndex{0};

  std::array<double,N> lastWaypoint;
};
