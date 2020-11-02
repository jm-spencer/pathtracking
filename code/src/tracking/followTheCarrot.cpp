#include "tracking/followTheCarrot.hpp"
#include <cmath>

FollowTheCarrotTracker::FollowTheCarrotTracker(double ikP, double ispeedTarget, double ilookaheadDist):
  kP(ikP), speedTarget(ispeedTarget), LookaheadTracker(ilookaheadDist) {
    std::get<0>(output) = speedTarget;
    reset();
}

std::tuple<double,double> FollowTheCarrotTracker::step(std::array<double,6> ireading) {
  if(!disabled) {
    std::copy(ireading.begin(), ireading.end(), lastReading.begin());

    std::array<double,2> goalPoint = getGoalPoint(ireading[0], ireading[1]);
    error[0] = goalPoint[0] - ireading[0];
    error[1] = goalPoint[1] - ireading[1];

    std::get<1>(output) = kP * (atan2(error[1], error[0]) - ireading[2]);

  }

  return output;
}

bool FollowTheCarrotTracker::isSettled() {
  return false;
  // Needs implementation in path generator. Possibly use a point defined
  // as NaN to signify end of path.
}

void FollowTheCarrotTracker::reset() {
  target = nullptr;
  lastWaypoint = {0,0};
  activeWaypoint = {0,0};
  lastReading = {0,0,0,0,0,0};
  error = {0,0,0,0,0,0};
  output = {0,0};
}

void FollowTheCarrotTracker::disable(bool iisDisabled) {
  disabled = iisDisabled;

  if(disabled){
    std::get<0>(output) = 0;
    std::get<1>(output) = 0;
  }else{
    std::get<0>(output) = speedTarget;
  }
}
