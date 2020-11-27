#include "tracking/followTheCarrot.hpp"
#include <cmath>

FollowTheCarrotTracker::FollowTheCarrotTracker(double ikP, double ispeedTarget, double ilookaheadDist):
  kP(ikP), speedTarget(ispeedTarget), LookaheadTracker(ilookaheadDist) {
    reset();
}

std::tuple<double,double> FollowTheCarrotTracker::step(std::array<double,6> ireading) {
  if(finished) {
    return {0,0};
  }

  if(!disabled) {
    std::copy(ireading.begin(), ireading.end(), lastReading.begin());

    std::array<double,2> &&goalPoint = getGoalPoint(ireading[0], ireading[1]);

    if (std::isnan(goalPoint[0])) {
      finished = true;
      return {0,0};
    }

    error[0] = goalPoint[0] - ireading[0];
    error[1] = goalPoint[1] - ireading[1];

    double e = std::fmod(atan2(error[1], error[0]) - ireading[2], 2 * M_PI);

    if(std::abs(e) > M_PI){
      e += e > 0 ? -2 * M_PI : 2 * M_PI;
    }

    output = {speedTarget, kP * e};
  }else{
    output = {0,0};
  }

  return output;
}

bool FollowTheCarrotTracker::isSettled() {
  return finished;
}

void FollowTheCarrotTracker::reset() {
  target = nullptr;
  finished = false;
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
