#include "tracking/ramsete.hpp"
#include <cmath>

RamseteTracker::RamseteTracker(double izeta, double ib, double ispeedTarget, double ilookaheadDist):
  zeta(izeta), b(ib), speedTarget(ispeedTarget), LookaheadTracker(ilookaheadDist) {
    reset();
}

std::tuple<double,double> RamseteTracker::step(std::array<double,6> ireading) {
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

    double omega = speedTarget * goalPoint[3];

    double k1 = 2 * zeta * sqrt(omega * omega + b * speedTarget * speedTarget);
    double dTheta = std::fmod(goalPoint[2] - ireading[2], 2 * M_PI);

    if(std::abs(dTheta) > M_PI){
      dTheta += dTheta > 0 ? -2 * M_PI : 2 * M_PI;
    }

    output = {
      speedTarget * cos(dTheta) + k1 * ((goalPoint[0] - ireading[0]) * cos(ireading[2]) + (goalPoint[1] - ireading[1]) * sin(ireading[2])),
      omega + b * speedTarget * (sin(dTheta) / dTheta) * ((goalPoint[1] - ireading[1]) * cos(ireading[2]) - (goalPoint[0] - ireading[0]) * sin(ireading[2])) + k1 * dTheta
    };
  } else {
    output = {0,0};
  }

  return output;
}

bool RamseteTracker::isSettled() {
  return finished;
}

void RamseteTracker::reset() {
  target = nullptr;
  finished = false;
  lastWaypoint = {0,0};
  activeWaypoint = {0,0};
  lastReading = {0,0,0,0,0,0};
  error = {0,0,0,0,0,0};
  output = {0,0};
}

void RamseteTracker::disable(bool iisDisabled) {
  disabled = iisDisabled;
}
