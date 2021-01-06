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

    std::array<double,2> &&goalPoint = getLateralError(ireading[0], ireading[1]);

    if (std::isnan(goalPoint[0])) {
      finished = true;
      return {0,0};
    }

    std::copy(goalPoint.begin(), goalPoint.end(), error.begin());

    double dTheta = std::fmod(goalPoint[1] - ireading[2], 2 * M_PI);

    if(std::abs(dTheta) > M_PI){
      dTheta += dTheta > 0 ? -2 * M_PI : 2 * M_PI;
    }

    double omega = (speedTarget / l) * tan(std::clamp(dTheta + atan(-k * goalPoint[0] / ireading[3]), -M_PI_2, M_PI_2));

    if(std::isnan(omega)){
      omega = 0;
    }

//    std::cout << dTheta << " " << -atan(k * goalPoint[0] / ireading[3]) << " " << omega << '\t';

    output = {speedTarget, omega};
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
