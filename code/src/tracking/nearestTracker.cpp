#include "tracking/nearestTracker.hpp"
#include <algorithm>

template<> std::array<double,2> NearestTracker<2>::getGoalPoint(double robotx, double roboty){
  double deltaRX = robotx - lastWaypoint[0];
  double deltaRY = roboty - lastWaypoint[1];

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
  double deltaRX = robotx - lastWaypoint[0];
  double deltaRY = roboty - lastWaypoint[1];

  double deltaPX = activeWaypoint[0] - lastWaypoint[0];
  double deltaPY = activeWaypoint[1] - lastWaypoint[1];

  double projScalar = (deltaRX * deltaPX + deltaRY * deltaPY) / (deltaPX * deltaPX + deltaPY * deltaPY);

  if(projScalar > 1 || std::isnan(projScalar)){
    std::copy(activeWaypoint.begin(), activeWaypoint.end(), lastWaypoint.begin());
    activeWaypoint = pathFile->get();
    waypointIndex++;
    return getGoalPoint(robotx, roboty);
  }

  double deltaPTheta = std::fmod(activeWaypoint[2] - lastWaypoint[2], 2 * M_PI);

  if(std::abs(deltaPTheta) > M_PI){
    deltaPTheta += deltaPTheta > 0 ? -2 * M_PI : 2 * M_PI;
  }

  return {lastWaypoint[0] + projScalar * deltaPX,
          lastWaypoint[1] + projScalar * deltaPY,
          lastWaypoint[2] + std::clamp(projScalar, 0.0, 1.0) * deltaPTheta};
}

template<> std::array<double,4> NearestTracker<4>::getGoalPoint(double robotx, double roboty){
  double deltaRX = robotx - lastWaypoint[0];
  double deltaRY = roboty - lastWaypoint[1];

  double deltaPX = activeWaypoint[0] - lastWaypoint[0];
  double deltaPY = activeWaypoint[1] - lastWaypoint[1];

  double projScalar = (deltaRX * deltaPX + deltaRY * deltaPY) / (deltaPX * deltaPX + deltaPY * deltaPY);

  if(projScalar > 1 || std::isnan(projScalar)){
    std::copy(activeWaypoint.begin(), activeWaypoint.end(), lastWaypoint.begin());
    activeWaypoint = pathFile->get();
    waypointIndex++;
    return getGoalPoint(robotx, roboty);
  }

  double deltaPTheta = std::fmod(activeWaypoint[2] - lastWaypoint[2], 2 * M_PI);

  if(std::abs(deltaPTheta) > M_PI){
    deltaPTheta += deltaPTheta > 0 ? -2 * M_PI : 2 * M_PI;
  }

  double deltaPGamma = activeWaypoint[3] - lastWaypoint[3];

  return {lastWaypoint[0] + projScalar * deltaPX,
          lastWaypoint[1] + projScalar * deltaPY,
          lastWaypoint[2] + std::clamp(projScalar, 0.0, 1.0) * deltaPTheta,
          lastWaypoint[3] + std::clamp(projScalar, 0.0, 1.0) * deltaPGamma};
}
