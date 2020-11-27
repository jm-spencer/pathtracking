#include "tracking/stanley.hpp"
#include <cmath>

StanleyTracker::StanleyTracker(double ik, double il, double ispeedTarget):
  k(ik), l(il), speedTarget(ispeedTarget){
    reset();
}

std::tuple<double,double> StanleyTracker::step(std::array<double,6> ireading){
  if(finished) {
    return {0,0};
  }

  if(!disabled) {
    std::copy(ireading.begin(), ireading.end(), lastReading.begin());

    std::array<double,3> &&goalPoint = getGoalPoint(ireading[0], ireading[1]);

    if (std::isnan(goalPoint[0])) {
      finished = true;
      return {0,0};
    }

    std::copy(goalPoint.begin(), goalPoint.end(), error.begin());

    double e = sqrt((goalPoint[0] - ireading[0]) * (goalPoint[0] - ireading[0]) + (goalPoint[1] - ireading[1]) * (goalPoint[1] - ireading[1]));

    output = {speedTarget,
              (ireading[3] / l) *
                tan(goalPoint[2] - ireading[2] +
                  atan(k * e / ireading[3]))};
  } else {
    output = {0,0};
  }

  return output;
}

bool StanleyTracker::isSettled() {
  return finished;
}

void StanleyTracker::reset() {
  target = nullptr;
  finished = false;
  lastWaypoint = {0,0};
  activeWaypoint = {0,0};
  lastReading = {0,0,0,0,0,0};
  error = {0,0,0,0,0,0};
  output = {0,0};
}

void StanleyTracker::disable(bool iisDisabled) {
  disabled = iisDisabled;
}

StanleyTracker st(1,1,1);
