#include "tracking/nearestTracker.hpp"

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

  return {lastWaypoint[0] + projScalar * deltaPX,
          lastWaypoint[1] + projScalar * deltaPY,
          atan2(deltaPY, deltaPX)};
}
