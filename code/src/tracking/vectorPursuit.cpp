#include "tracking/vectorPursuit.hpp"
#include <cmath>

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

    double cosTheta = cos(ireading[2]);
    double sinTheta = sin(ireading[2]);

    double screwx, screwy;

    if(goalPointLocal[1] != 0){

      double phi = atan2(2 * goalPointLocal[1] * goalPointLocal[1] - lookaheadDistSqr, 2 * goalPointLocal[0] * goalPointLocal[1]) - atan2(lookaheadDistSqr / 2 * goalPointLocal[1], 0);

      double r2$ = ((k * phi) / ((k-1) * phi + goalPoint[2] - ireading[2])) * (lookaheadDistSqr / 2 * goalPointLocal[1]);

      screwx = ireading[0] - r2$ * cosTheta;
      screwy = ireading[1] + r2$ * sinTheta;

    }else{

      screwx = ireading[0] - k * ((goalPoint[1] - ireading[1]) / (goalPoint[2] - ireading[2]));
      screwy = ireading[1] + k * ((goalPoint[0] - ireading[0]) / (goalPoint[2] - ireading[2]));

    }

    output = {speedTarget,
              ireading[3] / (ireading[1] * cosTheta - ireading[0] * sinTheta + screwx * sinTheta - screwy * cosTheta)};

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
