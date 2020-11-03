#include "tracking/purePursuit.hpp"
#include <cmath>

PurePursuitTracker::PurePursuitTracker(double ispeedTarget, double ilookaheadDist):
  speedTarget(ispeedTarget), LookaheadTracker(ilookaheadDist) {
    std::get<0>(output) = speedTarget;
    reset();
}

std::tuple<double,double> PurePursuitTracker::step(std::array<double,6> ireading) {
  if(!disabled) {
    std::copy(ireading.begin(), ireading.end(), lastReading.begin());

    std::array<double,2> &&goalPoint = globalToLocalCoords(getGoalPoint(ireading[0], ireading[1]), ireading);
    std::copy(goalPoint.begin(), goalPoint.end(), error.begin());

    std::get<1>(output) = (2 * goalPoint[1] * speedTarget) / (lookaheadDistSqr);
  }

  return output;
}

bool PurePursuitTracker::isSettled() {
  return false;
  // Needs implementation in path generator. Possibly use a point defined
  // as NaN to signify end of path.
}

void PurePursuitTracker::reset() {
  target = nullptr;
  lastWaypoint = {0,0};
  activeWaypoint = {0,0};
  lastReading = {0,0,0,0,0,0};
  error = {0,0,0,0,0,0};
  output = {0,0};
}

void PurePursuitTracker::disable(bool iisDisabled) {
  disabled = iisDisabled;

  if(disabled){
    std::get<0>(output) = 0;
    std::get<1>(output) = 0;
  }else{
    std::get<0>(output) = speedTarget;
  }
}
