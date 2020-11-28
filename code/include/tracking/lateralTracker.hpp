#pragma once
#include "kappa/api.hpp"
#include "tracking/abstractTracker.hpp"

template <std::size_t N>
class LateralTracker: public AbstractTracker<N> {
public:
  LateralTracker(){
    lastWaypoint.fill(0);
  }

  std::array<double,N-1> getLateralError(double robotx, double roboty);

protected:
  double waypointIndex{0};

  std::array<double,N> lastWaypoint;
};
