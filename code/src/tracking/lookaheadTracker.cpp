#include "tracking/lookaheadTracker.hpp"

template<> std::array<double,2> LookaheadTracker<2>::getGoalPoint(double robotx, double roboty, double effectiveLookaheadSqr){
  if(activeWaypoint[0] * activeWaypoint[0] + activeWaypoint[1] * activeWaypoint[1] < effectiveLookaheadSqr){
    std::copy(activeWaypoint.begin(), activeWaypoint.end(), lastWaypoint.begin());
    activeWaypoint = pathFile->get();
    return getGoalPoint(robotx, roboty, effectiveLookaheadSqr);
  }

  double deltaPX = activeWaypoint[0] - lastWaypoint[0];
  double deltaPY = activeWaypoint[1] - lastWaypoint[1];
  double deltaRX = activeWaypoint[0] - robotx;
  double deltaRY = activeWaypoint[1] - roboty;

  double a = deltaPX * deltaPX + deltaPY * deltaPY;
  double b = deltaPX * deltaRX + deltaPY * deltaRY;
  double c = deltaRX * deltaRX + deltaRY * deltaRY;

  double lambda = (sqrt(b * b - a * c) - b) / a;

  return {activeWaypoint[0] + lambda * deltaPX,
          activeWaypoint[1] + lambda * deltaPY};
}

template<> std::array<double,3> LookaheadTracker<3>::getGoalPoint(double robotx, double roboty, double effectiveLookaheadSqr){
  if(activeWaypoint[0] * activeWaypoint[0] + activeWaypoint[1] * activeWaypoint[1] < effectiveLookaheadSqr){
    std::copy(activeWaypoint.begin(), activeWaypoint.end(), lastWaypoint.begin());
    activeWaypoint = pathFile->get();
    return getGoalPoint(robotx, roboty, effectiveLookaheadSqr);
  }

  double deltaPX = activeWaypoint[0] - lastWaypoint[0];
  double deltaPY = activeWaypoint[1] - lastWaypoint[1];
  double deltaRX = activeWaypoint[0] - robotx;
  double deltaRY = activeWaypoint[1] - roboty;

  double a = deltaPX * deltaPX + deltaPY * deltaPY;
  double b = deltaPX * deltaRX + deltaPY * deltaRY;
  double c = deltaRX * deltaRX + deltaRY * deltaRY;

  double lambda = (sqrt(b * b - a * c) - b) / a;

  return {activeWaypoint[0] + lambda * deltaPX,
          activeWaypoint[1] + lambda * deltaPY,
          atan2(deltaPY, deltaPX)};
}