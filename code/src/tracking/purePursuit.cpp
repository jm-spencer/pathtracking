#include "tracking/purePursuit.hpp"
#include <cmath>

PurePursuitTracker::PurePursuitTracker(double ispeedTarget, double ilookaheadDist):
  speedTarget(ispeedTarget), LookaheadTracker(ilookaheadDist) {
    reset();
}

std::tuple<double,double> PurePursuitTracker::step(std::array<double,6> ireading) {
  if(finished) {
    return {0,0};
  }

  if(!disabled) {
    std::copy(ireading.begin(), ireading.end(), lastReading.begin());

    std::array<double,3> &&goalPoint = globalToLocalCoords(getGoalPoint(ireading[0], ireading[1]), ireading);

    if (std::isnan(goalPoint[0])) {
      finished = true;
      return {0,0};
    }

    std::copy(goalPoint.begin(), goalPoint.end(), error.begin());

    output = {speedTarget, (2 * goalPoint[1] * ireading[3]) / (lookaheadDistSqr)};
  } else {
    output = {0,0};
  }

  return output;
}

bool PurePursuitTracker::isSettled() {
  return finished;
}

void PurePursuitTracker::reset() {
  target = nullptr;
  finished = false;
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
