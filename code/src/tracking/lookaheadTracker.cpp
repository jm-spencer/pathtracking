#include "tracking/lookaheadTracker.hpp"

template<> std::array<double,2> LookaheadTracker<2>::getGoalPoint(double robotx, double roboty, double effectiveLookaheadSqr){
  double deltaRX = activeWaypoint[0] - robotx;
  double deltaRY = activeWaypoint[1] - roboty;

  double deltaPX = activeWaypoint[0] - lastWaypoint[0];
  double deltaPY = activeWaypoint[1] - lastWaypoint[1];

  double b = deltaPX * deltaRX + deltaPY * deltaRY;
  double c = deltaRX * deltaRX + deltaRY * deltaRY - effectiveLookaheadSqr;

  if(c < 0 || b < 0){
    std::copy(activeWaypoint.begin(), activeWaypoint.end(), lastWaypoint.begin());
    activeWaypoint = pathFile->get();
    waypointIndex++;
    return getGoalPoint(robotx, roboty, effectiveLookaheadSqr);
  }

  double a = deltaPX * deltaPX + deltaPY * deltaPY;

  double lambda = (sqrt(b * b - a * c) - b) / a;

  if(std::isnan(lambda)){
    lambda = -1;
  }

  return {activeWaypoint[0] + lambda * deltaPX,
          activeWaypoint[1] + lambda * deltaPY};
}

template<> std::array<double,3> LookaheadTracker<3>::getGoalPoint(double robotx, double roboty, double effectiveLookaheadSqr){
  double deltaRX = activeWaypoint[0] - robotx;
  double deltaRY = activeWaypoint[1] - roboty;

  double deltaPX = activeWaypoint[0] - lastWaypoint[0];
  double deltaPY = activeWaypoint[1] - lastWaypoint[1];

  double b = deltaPX * deltaRX + deltaPY * deltaRY;
  double c = deltaRX * deltaRX + deltaRY * deltaRY - effectiveLookaheadSqr;

  if(c < 0 || b < 0){
    std::copy(activeWaypoint.begin(), activeWaypoint.end(), lastWaypoint.begin());
    activeWaypoint = pathFile->get();
    waypointIndex++;
    return getGoalPoint(robotx, roboty, effectiveLookaheadSqr);
  }

  double a = deltaPX * deltaPX + deltaPY * deltaPY;

  double lambda = (sqrt(b * b - a * c) - b) / a;

  if(std::isnan(lambda)){
    lambda = -1;
  }

  //std::cout << lambda << " " << waypointIndex << "\tG: (" << activeWaypoint[0] + lambda * deltaPX << ", " << activeWaypoint[1] + lambda * deltaPY << ")\t";

  double theta;
  // if the angle "wraps around" from +PI to -PI over this path segment, theta must be adjusted
  if(std::abs(activeWaypoint[2]) > M_PI_2 && (activeWaypoint[2] > 0 != lastWaypoint[2] > 0)){
    if(lastWaypoint[2] > 0){
      theta = (lambda + 1) * (activeWaypoint[2] + M_2_PI) - lambda * lastWaypoint[2];
    }else{
      theta = (lambda + 1) * activeWaypoint[2] - lambda * (lastWaypoint[2] + M_2_PI);
    }
  }else{
    theta = (lambda + 1) * activeWaypoint[2] - lambda * lastWaypoint[2];
  }

  return {activeWaypoint[0] + lambda * deltaPX,
          activeWaypoint[1] + lambda * deltaPY,
          theta};
}

template<> std::array<double,4> LookaheadTracker<4>::getGoalPoint(double robotx, double roboty, double effectiveLookaheadSqr){
  double deltaRX = activeWaypoint[0] - robotx;
  double deltaRY = activeWaypoint[1] - roboty;

  double deltaPX = activeWaypoint[0] - lastWaypoint[0];
  double deltaPY = activeWaypoint[1] - lastWaypoint[1];

  double b = deltaPX * deltaRX + deltaPY * deltaRY;
  double c = deltaRX * deltaRX + deltaRY * deltaRY - effectiveLookaheadSqr;

  if(c < 0 || b < 0){
    std::copy(activeWaypoint.begin(), activeWaypoint.end(), lastWaypoint.begin());
    activeWaypoint = pathFile->get();
    waypointIndex++;
    return getGoalPoint(robotx, roboty, effectiveLookaheadSqr);
  }

  double a = deltaPX * deltaPX + deltaPY * deltaPY;

  double lambda = (sqrt(b * b - a * c) - b) / a;

  if(std::isnan(lambda)){
    lambda = -1;
  }

  //std::cout << lambda << " " << waypointIndex << "\tG: (" << activeWaypoint[0] + lambda * deltaPX << ", " << activeWaypoint[1] + lambda * deltaPY << ")\t";

  double theta;
  // if the angle "wraps around" from +PI to -PI over this path segment, theta must be adjusted
  if(std::abs(activeWaypoint[2]) > M_PI_2 && (activeWaypoint[2] > 0 != lastWaypoint[2] > 0)){
    if(lastWaypoint[2] > 0){
      theta = (lambda + 1) * (activeWaypoint[2] + M_2_PI) - lambda * lastWaypoint[2];
    }else{
      theta = (lambda + 1) * activeWaypoint[2] - lambda * (lastWaypoint[2] + M_2_PI);
    }
  }else{
    theta = (lambda + 1) * activeWaypoint[2] - lambda * lastWaypoint[2];
  }

  return {activeWaypoint[0] + lambda * deltaPX,
          activeWaypoint[1] + lambda * deltaPY,
          theta,
          (lambda + 1) * activeWaypoint[3] - lambda * lastWaypoint[3]};
}

template<> std::array<double,2> LookaheadTracker<2>::globalToLocalCoords(const std::array<double,2> &point, const std::array<double,6> &basis) {
  double deltaXg = point[0] - basis[0];
  double deltaYg = point[1] - basis[1];

  double sinTheta = sin(basis[2]);
  double cosTheta = cos(basis[2]);

  return {deltaXg * cosTheta + deltaYg * sinTheta,
          deltaYg * cosTheta - deltaXg * sinTheta};
}

template<> std::array<double,3> LookaheadTracker<3>::globalToLocalCoords(const std::array<double,3> &point, const std::array<double,6> &basis) {
  double deltaXg = point[0] - basis[0];
  double deltaYg = point[1] - basis[1];

  double sinTheta = sin(basis[2]);
  double cosTheta = cos(basis[2]);

  //std::cout << "G': (" << deltaXg * cosTheta + deltaYg * sinTheta << ", " << deltaYg * cosTheta - deltaXg * sinTheta << ")\t";

  return {deltaXg * cosTheta + deltaYg * sinTheta,
          deltaYg * cosTheta - deltaXg * sinTheta,
          point[2] - basis[2]};
}

template<> std::array<double,4> LookaheadTracker<4>::globalToLocalCoords(const std::array<double,4> &point, const std::array<double,6> &basis) {
  double deltaXg = point[0] - basis[0];
  double deltaYg = point[1] - basis[1];

  double sinTheta = sin(basis[2]);
  double cosTheta = cos(basis[2]);

  return {deltaXg * cosTheta + deltaYg * sinTheta,
          deltaYg * cosTheta - deltaXg * sinTheta,
          point[2] - basis[2],
          point[3]};
}
