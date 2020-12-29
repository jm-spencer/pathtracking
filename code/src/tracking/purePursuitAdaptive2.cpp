#include "tracking/purePursuitAdaptive2.hpp"
#include "kappa/api.hpp"

PurePursuitAdaptive2Tracker::PurePursuitAdaptive2Tracker(uint iN, double ispeedTarget, double ilookaheadDist):
  n(iN), speedTarget(ispeedTarget), lookaheadDist(ilookaheadDist), lookaheadDistSqr(ilookaheadDist * ilookaheadDist){
    //reset();
}

void PurePursuitAdaptive2Tracker::setTarget(const uint &itarget) {
  finished = false;
  this->reset();

  switch(itarget){
    case 1:
      pathFileL = std::make_shared<kappa::BinFileInput<double,2>>("/usd/paths/path1.2");
      pathFileN = std::make_shared<kappa::BinFileInput<double,3>>("/usd/paths/path1.3");
      pathFileC = std::make_shared<kappa::BinFileInput<double,3>>("/usd/paths/path1.3");
      break;

    case 2:
      pathFileL = std::make_shared<kappa::BinFileInput<double,2>>("/usd/paths/path2.2");
      pathFileN = std::make_shared<kappa::BinFileInput<double,3>>("/usd/paths/path2.3");
      pathFileC = std::make_shared<kappa::BinFileInput<double,3>>("/usd/paths/path2.3");
      break;

    case 3:
      pathFileL = std::make_shared<kappa::BinFileInput<double,2>>("/usd/paths/path3.2");
      pathFileN = std::make_shared<kappa::BinFileInput<double,3>>("/usd/paths/path3.3");
      pathFileC = std::make_shared<kappa::BinFileInput<double,3>>("/usd/paths/path3.3");
      break;

  }

  for(uint i = 0; i < n; i++){
    pathFileC->get();
  }

  lastLookaheadWaypoint   = pathFileL->get();
  lastNearestWaypoint     = pathFileN->get();

  activeLookaheadWaypoint = pathFileL->get();
  activeNearestWaypoint   = pathFileN->get();
  activeCurvatureWaypoint = pathFileC->get();
}

std::tuple<double,double> PurePursuitAdaptive2Tracker::step(std::array<double,6> ireading) {
  if(finished) {
    return {0,0};
  }

  if(!disabled) {
    std::copy(ireading.begin(), ireading.end(), lastReading.begin());

    double curvature = getCurvature(ireading[0], ireading[1]);
    double adaptiveLookaheadDistSqr = lookaheadDistSqr / ((1 + curvature * lookaheadDist) * (1 + curvature * lookaheadDist));

    std::array<double,2> &&goalPoint = globalToLocalCoords(getGoalPoint(ireading[0], ireading[1], adaptiveLookaheadDistSqr), ireading);

    if (std::isnan(goalPoint[0])) {
      finished = true;
      return {0,0};
    }

    std::copy(goalPoint.begin(), goalPoint.end(), error.begin());

    output = {speedTarget, (2 * goalPoint[1] * speedTarget) / (adaptiveLookaheadDistSqr)};
  } else {
    output = {0,0};
  }

  return output;
}

bool PurePursuitAdaptive2Tracker::isSettled() {
  return finished;
}

void PurePursuitAdaptive2Tracker::reset() {
  target = 0;
  finished = false;
  lastLookaheadWaypoint = {0,0};
  activeLookaheadWaypoint = {0,0};
  lastNearestWaypoint = {0,0,0};
  activeNearestWaypoint = {0,0,0};
  activeCurvatureWaypoint = {0,0,0};
  lastReading = {0,0,0,0,0,0};
  error = {0,0,0,0,0,0};
  output = {0,0};
}

void PurePursuitAdaptive2Tracker::disable(bool iisDisabled) {
  disabled = iisDisabled;

  if(disabled){
    std::get<0>(output) = 0;
    std::get<1>(output) = 0;
  }else{
    std::get<0>(output) = speedTarget;
  }
}

std::array<double,2> PurePursuitAdaptive2Tracker::getGoalPoint(double robotx, double roboty, double adaptLookaheadDistSqr) {
  double deltaRX = activeLookaheadWaypoint[0] - robotx;
  double deltaRY = activeLookaheadWaypoint[1] - roboty;

  double deltaPX = activeLookaheadWaypoint[0] - lastLookaheadWaypoint[0];
  double deltaPY = activeLookaheadWaypoint[1] - lastLookaheadWaypoint[1];

  double b = deltaPX * deltaRX + deltaPY * deltaRY;
  double c = deltaRX * deltaRX + deltaRY * deltaRY - adaptLookaheadDistSqr;

  if(c < 0 || b < 0){
    std::copy(activeLookaheadWaypoint.begin(), activeLookaheadWaypoint.end(), lastLookaheadWaypoint.begin());
    activeLookaheadWaypoint = pathFileL->get();
    return getGoalPoint(robotx, roboty, adaptLookaheadDistSqr);
  }

  double a = deltaPX * deltaPX + deltaPY * deltaPY;

  double lambda = (sqrt(b * b - a * c) - b) / a;

  if(std::isnan(lambda)){
    lambda = -1;
  }

  std::cout << "(" << robotx << ", " << roboty << ")\t" << lambda << " " << adaptLookaheadDistSqr << "\tG: (" << activeLookaheadWaypoint[0] + lambda * deltaPX << ", " << activeLookaheadWaypoint[1] + lambda * deltaPY << ")\t";

  return {activeLookaheadWaypoint[0] + lambda * deltaPX,
          activeLookaheadWaypoint[1] + lambda * deltaPY};
}

double PurePursuitAdaptive2Tracker::getCurvature(double robotx, double roboty) {
  double deltaRX = robotx - lastNearestWaypoint[0];
  double deltaRY = roboty - lastNearestWaypoint[1];

  double deltaPX = activeNearestWaypoint[0] - lastNearestWaypoint[0];
  double deltaPY = activeNearestWaypoint[1] - lastNearestWaypoint[1];

  double magPsqr = deltaPX * deltaPX + deltaPY * deltaPY;

  double projScalar = (deltaRX * deltaPX + deltaRY * deltaPY) / magPsqr;

  if(projScalar > 1 || magPsqr == 0){
    std::copy(activeNearestWaypoint.begin(), activeNearestWaypoint.end(), lastNearestWaypoint.begin());
    activeNearestWaypoint = pathFileN->get();
    activeCurvatureWaypoint = pathFileC->get();
    return getCurvature(robotx, roboty);
  }

  if (std::isnan(activeCurvatureWaypoint[0])) {
    finished = true;
    return 0;
  }

  double dTheta = std::abs(std::fmod(activeCurvatureWaypoint[2] - activeNearestWaypoint[2], 2*M_PI));

  if(dTheta > M_PI){
    dTheta = 2 * M_PI - dTheta;
  }

  std::cout << "dTheta:" << dTheta << "\t";

  return dTheta / (n * 5); // 5 (cm) = waypoint spacing
}

std::array<double,2> PurePursuitAdaptive2Tracker::globalToLocalCoords(const std::array<double,2> &point, const std::array<double,6> &basis) {
  double deltaXg = point[0] - basis[0];
  double deltaYg = point[1] - basis[1];

  double sinTheta = sin(basis[2]);
  double cosTheta = cos(basis[2]);

  return {deltaXg * cosTheta + deltaYg * sinTheta,
          deltaYg * cosTheta - deltaXg * sinTheta};
}

void PurePursuitAdaptive2Tracker::skipPoint(uint recurse){
  if(!recurse) return;

  std::copy(activeLookaheadWaypoint.begin(), activeLookaheadWaypoint.end(), lastLookaheadWaypoint.begin());
  activeLookaheadWaypoint = pathFileL->get();
  skipPoint(recurse - 1);
}
