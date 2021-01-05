#include "tracking/vectorPursuit.hpp"
#include <cmath>

static inline double sgn(double val) {
    return val > 0 ? 1 : val < 0 ? -1 : 0;
}

VectorPursuitTracker::VectorPursuitTracker(double ik, double ispeedTarget, double ilookaheadDist):
  k(ik), speedTarget(ispeedTarget), LookaheadTracker(ilookaheadDist) {
    reset();
}

std::tuple<double,double> VectorPursuitTracker::step(std::array<double,6> ireading) {
  if(finished) {
    return {0,0};
  }

  if(!disabled) {
    std::copy(ireading.begin(), ireading.end(), lastReading.begin());

    std::array<double,3> goalPoint = getGoalPoint(ireading[0], ireading[1]);
    std::array<double,3> goalPointLocal = globalToLocalCoords(goalPoint, ireading);

    if (std::isnan(goalPoint[0])) {
      finished = true;
      return {0,0};
    }

    std::copy(goalPoint.begin(), goalPoint.end(), error.begin());

    double dSqr = goalPointLocal[0] * goalPointLocal[0] + goalPointLocal[1] * goalPointLocal[1];

    double dTheta = std::fmod(goalPointLocal[2], 2 * M_PI);

    if(std::abs(dTheta) > M_PI){
      dTheta += dTheta > 0 ? -2 * M_PI : 2 * M_PI;
    }

    if(goalPointLocal[1] == 0){
      if(dTheta == 0){
        output = {speedTarget, 0};
      } else {
        output = {speedTarget,
                  speedTarget * dTheta /
                    (k * ((goalPoint[1] - ireading[1]) * sin(ireading[2]) +
                          (goalPoint[0] - ireading[0]) * cos(ireading[2])))};
      }
    } else {
      double phi = sgn(goalPointLocal[1]) * (
                      atan2(2 * goalPointLocal[1] * goalPointLocal[1] - dSqr,
                            2 * goalPointLocal[0] * std::abs(goalPointLocal[1])) +
                        M_PI_2);

      output = {speedTarget,
            2 * speedTarget * goalPointLocal[1] * ((k-1) * phi + dTheta) / (k * phi * dSqr)};
    }

  } else {
    output = {0,0};
  }

  return output;
}

bool VectorPursuitTracker::isSettled() {
  return finished;
}

void VectorPursuitTracker::reset() {
  target = nullptr;
  finished = false;
  lastWaypoint = {0,0};
  activeWaypoint = {0,0};
  lastReading = {0,0,0,0,0,0};
  error = {0,0,0,0,0,0};
  output = {0,0};
}

void VectorPursuitTracker::disable(bool iisDisabled) {
  disabled = iisDisabled;

  if(disabled){
    std::get<0>(output) = 0;
    std::get<1>(output) = 0;
  }else{
    std::get<0>(output) = speedTarget;
  }
}
