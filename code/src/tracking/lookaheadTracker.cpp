#include "tracking/lookaheadTracker.hpp"

template<> std::array<double,2> LookaheadTracker<2>::getGoalPoint(double robotx, double roboty, double effectiveLookaheadSqr){
  double deltaRX = activeWaypoint[0] - robotx;
  double deltaRY = activeWaypoint[1] - roboty;
  double c = deltaRX * deltaRX + deltaRY * deltaRY - effectiveLookaheadSqr;

  if(c < 0){
    std::copy(activeWaypoint.begin(), activeWaypoint.end(), lastWaypoint.begin());
    activeWaypoint = pathFile->get();
    return getGoalPoint(robotx, roboty, effectiveLookaheadSqr);
  }

  double deltaPX = activeWaypoint[0] - lastWaypoint[0];
  double deltaPY = activeWaypoint[1] - lastWaypoint[1];

  double a = deltaPX * deltaPX + deltaPY * deltaPY;
  double b = deltaPX * deltaRX + deltaPY * deltaRY;

  double lambda = (sqrt(b * b - a * c) - b) / a;

  return {activeWaypoint[0] + lambda * deltaPX,
          activeWaypoint[1] + lambda * deltaPY};
}

template<> std::array<double,3> LookaheadTracker<3>::getGoalPoint(double robotx, double roboty, double effectiveLookaheadSqr){
  double deltaRX = activeWaypoint[0] - robotx;
  double deltaRY = activeWaypoint[1] - roboty;
  double c = deltaRX * deltaRX + deltaRY * deltaRY - effectiveLookaheadSqr;

  if(c < 0){
    std::copy(activeWaypoint.begin(), activeWaypoint.end(), lastWaypoint.begin());
    activeWaypoint = pathFile->get();
    return getGoalPoint(robotx, roboty, effectiveLookaheadSqr);
  }

  double deltaPX = activeWaypoint[0] - lastWaypoint[0];
  double deltaPY = activeWaypoint[1] - lastWaypoint[1];

  double a = deltaPX * deltaPX + deltaPY * deltaPY;
  double b = deltaPX * deltaRX + deltaPY * deltaRY;

  double lambda = (sqrt(b * b - a * c) - b) / a;

  return {activeWaypoint[0] + lambda * deltaPX,
          activeWaypoint[1] + lambda * deltaPY,
          atan2(deltaPY, deltaPX)};
}
