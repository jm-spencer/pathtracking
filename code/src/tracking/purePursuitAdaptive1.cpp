#include "tracking/purePursuitAdaptive1.hpp"
#include "kappa/api.hpp"

PurePursuitAdaptive1Tracker::PurePursuitAdaptive1Tracker(double ispeedTarget, double ilookaheadDist):
  speedTarget(ispeedTarget), lookaheadDist(ilookaheadDist), lookaheadDistSqr(ilookaheadDist * ilookaheadDist){
    //reset();
}

void PurePursuitAdaptive1Tracker::setTarget(const uint &itarget) {
  finished = false;

  this->reset();

  switch(itarget){
    case 1:
      pathFileL = std::make_shared<kappa::BinFileInput<double,2>>("/usd/paths/path1.2");
      pathFileN = std::make_shared<kappa::BinFileInput<double,2>>("/usd/paths/path1.2");
      break;

    case 2:
      pathFileL = std::make_shared<kappa::BinFileInput<double,2>>("/usd/paths/path2.2");
      pathFileN = std::make_shared<kappa::BinFileInput<double,2>>("/usd/paths/path2.2");
      break;

    case 3:
      pathFileL = std::make_shared<kappa::BinFileInput<double,2>>("/usd/paths/path3.2");
      pathFileN = std::make_shared<kappa::BinFileInput<double,2>>("/usd/paths/path3.2");
      break;

  }

  activeLookaheadWaypoint = pathFileL->get();
  activeNearestWaypoint   = pathFileN->get();
}

std::tuple<double,double> PurePursuitAdaptive1Tracker::step(std::array<double,6> ireading) {
  if(finished) {
    return {0,0};
  }

  if(!disabled) {
    std::copy(ireading.begin(), ireading.end(), lastReading.begin());

    double lateralError = getLateralError(ireading[0], ireading[1]);
    double adaptiveLookaheadDistSqr = lookaheadDistSqr + 2 * lookaheadDist * lateralError + lateralError * lateralError;

    std::array<double,2> &&goalPoint = globalToLocalCoords(getGoalPoint(ireading[0], ireading[1], adaptiveLookaheadDistSqr), ireading);

    if (std::isnan(goalPoint[0])) {
      finished = true;
      return {0,0};
    }

    std::copy(goalPoint.begin(), goalPoint.end(), error.begin());

    output = {speedTarget, (2 * goalPoint[1] * ireading[3]) / (adaptiveLookaheadDistSqr)};
  } else {
    output = {0,0};
  }

  return output;
}

bool PurePursuitAdaptive1Tracker::isSettled() {
  return finished;
}

void PurePursuitAdaptive1Tracker::reset() {
  target = 0;
  lastLookaheadWaypoint.fill(0);
  activeLookaheadWaypoint.fill(0);
  lastNearestWaypoint.fill(0);
  activeNearestWaypoint.fill(0);
  lastReading.fill(0);
  error.fill(0);
  output = {0,0};
}

void PurePursuitAdaptive1Tracker::disable(bool iisDisabled) {
  disabled = iisDisabled;

  if(disabled){
    std::get<0>(output) = 0;
    std::get<1>(output) = 0;
  }else{
    std::get<0>(output) = speedTarget;
  }
}

std::array<double,2> PurePursuitAdaptive1Tracker::getGoalPoint(double robotx, double roboty, double lookaheadDistSqr) {
  double deltaRX = activeLookaheadWaypoint[0] - robotx;
  double deltaRY = activeLookaheadWaypoint[1] - roboty;

  double deltaPX = activeLookaheadWaypoint[0] - lastLookaheadWaypoint[0];
  double deltaPY = activeLookaheadWaypoint[1] - lastLookaheadWaypoint[1];

  double b = deltaPX * deltaRX + deltaPY * deltaRY;
  double c = deltaRX * deltaRX + deltaRY * deltaRY - lookaheadDistSqr;

  if(c < 0 || b < 0){
    std::copy(activeLookaheadWaypoint.begin(), activeLookaheadWaypoint.end(), lastLookaheadWaypoint.begin());
    activeLookaheadWaypoint = pathFileL->get();
    return getGoalPoint(robotx, roboty, lookaheadDistSqr);
  }

  double a = deltaPX * deltaPX + deltaPY * deltaPY;

  double lambda = (sqrt(b * b - a * c) - b) / a;

  if(std::isnan(lambda)){
    lambda = -1;
  }

  // std::cout << "(" << robotx << ", " << roboty << ")\t" << lambda << " " << waypointIndex << "\tG: (" << activeLookaheadWaypoint[0] + lambda * deltaPX << ", " << activeLookaheadWaypoint[1] + lambda * deltaPY << ")\t";

  return {activeLookaheadWaypoint[0] + lambda * deltaPX,
          activeLookaheadWaypoint[1] + lambda * deltaPY};
}

double PurePursuitAdaptive1Tracker::getLateralError(double robotx, double roboty) {
  double deltaRX = robotx - lastNearestWaypoint[0];
  double deltaRY = roboty - lastNearestWaypoint[1];

  double deltaPX = activeNearestWaypoint[0] - lastNearestWaypoint[0];
  double deltaPY = activeNearestWaypoint[1] - lastNearestWaypoint[1];

  double magPsqr = deltaPX * deltaPX + deltaPY * deltaPY;

  double projScalar = (deltaRX * deltaPX + deltaRY * deltaPY) / magPsqr;

  if(projScalar > 1 || magPsqr == 0){
    std::copy(activeNearestWaypoint.begin(), activeNearestWaypoint.end(), lastNearestWaypoint.begin());
    activeNearestWaypoint = pathFileN->get();
    return getLateralError(robotx, roboty);
  }

  double dist = sqrt(deltaRX * deltaRX + deltaRY * deltaRY - (projScalar * projScalar * magPsqr));
  if(std::isnan(dist) && !std::isnan(deltaPX)){
    dist = 0; // can only occur if there is a rounding error significant enough to make "0" < 0
  }

  return dist;
}

std::array<double,2> PurePursuitAdaptive1Tracker::globalToLocalCoords(const std::array<double,2> &point, const std::array<double,6> &basis) {
  double deltaXg = point[0] - basis[0];
  double deltaYg = point[1] - basis[1];

  double sinTheta = sin(basis[2]);
  double cosTheta = cos(basis[2]);

  return {deltaXg * cosTheta + deltaYg * sinTheta,
          deltaYg * cosTheta - deltaXg * sinTheta};
}
