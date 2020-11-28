#include "tracking/lateralTracker.hpp"
#include <algorithm>

template<> std::array<double,1> LateralTracker<2>::getLateralError(double robotx, double roboty){
  double deltaRX = activeWaypoint[0] - robotx;
  double deltaRY = activeWaypoint[1] - roboty;

  double deltaPX = activeWaypoint[0] - lastWaypoint[0];
  double deltaPY = activeWaypoint[1] - lastWaypoint[1];

  double magPsqr = deltaPX * deltaPX + deltaPY * deltaPY;

  double projScalar = (deltaRX * deltaPX + deltaRY * deltaPY) / magPsqr;

  if(projScalar > 1){
    std::copy(activeWaypoint.begin(), activeWaypoint.end(), lastWaypoint.begin());
    activeWaypoint = pathFile->get();
    waypointIndex++;
    return getLateralError(robotx, roboty);
  }

  double dist = sqrt(deltaRX * deltaRX + deltaRY * deltaRY - (projScalar * projScalar * magPsqr));
  if(std::isnan(dist)){
    dist = 0; // can only occur if there is a rounding error significant enough to make "0" < 0
  }

  bool sign = deltaPX * deltaRY > deltaPY * deltaRX;

  return {sign ? dist : -dist};
}

template<> std::array<double,2> LateralTracker<3>::getLateralError(double robotx, double roboty){
  double deltaRX = activeWaypoint[0] - robotx;
  double deltaRY = activeWaypoint[1] - roboty;

  double deltaPX = activeWaypoint[0] - lastWaypoint[0];
  double deltaPY = activeWaypoint[1] - lastWaypoint[1];

  double magPsqr = deltaPX * deltaPX + deltaPY * deltaPY;

  double projScalar = (deltaRX * deltaPX + deltaRY * deltaPY) / magPsqr;

  if(projScalar > 1){
    std::copy(activeWaypoint.begin(), activeWaypoint.end(), lastWaypoint.begin());
    activeWaypoint = pathFile->get();
    waypointIndex++;
    return getLateralError(robotx, roboty);
  }

  double dist = sqrt(deltaRX * deltaRX + deltaRY * deltaRY - (projScalar * projScalar * magPsqr));
  if(std::isnan(dist)){
    dist = 0; // can only occur if there is a rounding error significant enough to make "0" < 0
  }

  bool sign = deltaPX * deltaRY > deltaPY * deltaRX;

  double deltaPTheta = std::fmod(activeWaypoint[2] - lastWaypoint[2], 2 * M_PI);

  if(std::abs(deltaPTheta) > M_PI){
    deltaPTheta += deltaPTheta > 0 ? -2 * M_PI : 2 * M_PI;
  }

  return {sign ? dist : -dist, lastWaypoint[2] + std::clamp(projScalar, 0.0, 1.0) * deltaPTheta};
}
