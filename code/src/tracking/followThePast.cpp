#include "tracking/followThePast.hpp"
#include <cmath>

FollowThePastTracker::FollowThePastTracker(double il, double ilookaheadDist, double ispeedTarget):
  l(il), lookaheadDist(ilookaheadDist), speedTarget(ispeedTarget){
    reset();
}

std::tuple<double,double> FollowThePastTracker::step(std::array<double,6> ireading){
  if(finished) {
    return {0,0};
  }

  if(!disabled) {
    std::copy(ireading.begin(), ireading.end(), lastReading.begin());

    std::array<double,4> &&goalPoint = getGoalPoint(ireading[0], ireading[1]);

    if (std::isnan(goalPoint[0])) {
      finished = true;
      return {0,0};
    }

    std::copy(goalPoint.begin(), goalPoint.end(), error.begin());

    double delta = goalPoint[2] + atan(goalPoint[3] * l);
    double phi_a = atan2(goalPoint[1] - ireading[1] + lookaheadDist * sin(delta),
                         goalPoint[0] - ireading[0] + lookaheadDist * cos(delta)
                       );

    double phi_t = std::fmod(phi_a - ireading[2], 2 * M_PI);

    if(std::abs(phi_t) > M_PI){
        phi_t += phi_t > 0 ? -2 * M_PI : 2 * M_PI;
    }

    std::cout << "(" << ireading[0] << ", " << ireading[1] << ")\t(" << goalPoint[0] << ", " << goalPoint[1] << ")\t" << phi_a << " " << ireading[2] << " " << phi_t;

    output = {speedTarget,
              (ireading[3] / l) * tan(std::clamp(phi_t, -M_PI_2, M_PI_2))};
  } else {
    output = {0,0};
  }

  return output;
}

bool FollowThePastTracker::isSettled() {
  return finished;
}

void FollowThePastTracker::reset() {
  target = nullptr;
  finished = false;
  lastWaypoint = {0,0};
  activeWaypoint = {0,0};
  lastReading = {0,0,0,0,0,0};
  error = {0,0,0,0,0,0};
  output = {0,0};
}

void FollowThePastTracker::disable(bool iisDisabled) {
  disabled = iisDisabled;
}
