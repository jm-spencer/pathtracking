#include "tracking/nearestTracker.hpp"
#include <algorithm>

template<> std::array<double,2> NearestTracker<2>::getGoalPoint(double robotx, double roboty){
  double deltaRX = activeWaypoint[0] - robotx;
  double deltaRY = activeWaypoint[1] - roboty;

  double deltaPX = activeWaypoint[0] - lastWaypoint[0];
  double deltaPY = activeWaypoint[1] - lastWaypoint[1];

  double projScalar = (deltaRX * deltaPX + deltaRY * deltaPY) / (deltaPX * deltaPX + deltaPY * deltaPY);

  if(projScalar > 1){
    std::copy(activeWaypoint.begin(), activeWaypoint.end(), lastWaypoint.begin());
    activeWaypoint = pathFile->get();
    waypointIndex++;
    return getGoalPoint(robotx, roboty);
  }

  return {lastWaypoint[0] + projScalar * deltaPX,
          lastWaypoint[1] + projScalar * deltaPY};
}

template<> std::array<double,3> NearestTracker<3>::getGoalPoint(double robotx, double roboty){
  double deltaRX = activeWaypoint[0] - robotx;
  double deltaRY = activeWaypoint[1] - roboty;

  double deltaPX = activeWaypoint[0] - lastWaypoint[0];
  double deltaPY = activeWaypoint[1] - lastWaypoint[1];

  double projScalar = (deltaRX * deltaPX + deltaRY * deltaPY) / (deltaPX * deltaPX + deltaPY * deltaPY);

  if(projScalar > 1){
    std::copy(activeWaypoint.begin(), activeWaypoint.end(), lastWaypoint.begin());
    activeWaypoint = pathFile->get();
    waypointIndex++;
    return getGoalPoint(robotx, roboty);
  }

  double theta;
  // if the angle "wraps around" from +PI to -PI over this path segment, theta must be adjusted
  if(std::abs(activeWaypoint[2]) > M_PI_2 && (activeWaypoint[2] > 0 != lastWaypoint[2] > 0)){
    if(lastWaypoint[2] > 0){
      theta = std::clamp(projScalar, 0.0, 1.0) * (activeWaypoint[2] + M_2_PI) + (1 - std::clamp(projScalar, 0.0, 1.0)) * lastWaypoint[2];
    }else{
      theta = std::clamp(projScalar, 0.0, 1.0) * activeWaypoint[2] + (1 - std::clamp(projScalar, 0.0, 1.0)) * (lastWaypoint[2] + M_2_PI);
    }
  }else{
    theta = std::clamp(projScalar, 0.0, 1.0) * activeWaypoint[2] + (1 - std::clamp(projScalar, 0.0, 1.0)) * lastWaypoint[2];
  }

  return {lastWaypoint[0] + projScalar * deltaPX,
          lastWaypoint[1] + projScalar * deltaPY,
          atan2(deltaPY, deltaPX)};
}
