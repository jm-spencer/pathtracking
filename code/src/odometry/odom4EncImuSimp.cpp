#include "odometry/odom4EncImuSimp.hpp"

Odom4EncImuSimp::Odom4EncImuSimp(OdomVals &&ivals,
              std::unique_ptr<okapi::Filter> ivelFilter,
              std::unique_ptr<okapi::Filter> istfVelFilter,
              std::unique_ptr<okapi::Filter> iangVelFilter,
              std::shared_ptr<kappa::AbstractInput<std::array<double,5>>> iinput):
              input(iinput), vals(ivals), velFilter(std::move(ivelFilter)),
              stfVelFilter(std::move(istfVelFilter)),
              angVelFilter(std::move(iangVelFilter))
  {
    lastIn[5] = pros::millis();
  }

const std::array<double,6> &Odom4EncImuSimp::step() {
  const std::array<double,5> &in = input->get();


  double dL = in[0] - lastIn[0];
  double dB = in[1] - lastIn[1];
  double dR = in[2] - lastIn[2];
  double dF = in[3] - lastIn[3];
  double dT = pros::millis() - lastIn[5];

  if(dT == 0) return pose;

  lastIn[5] = pros::millis();
  lastIn[0] = in[0];
  lastIn[1] = in[1];
  lastIn[2] = in[2];
  lastIn[3] = in[3];

  double dTheta;

  if(in[4] != lastIn[4]){
    dTheta = in[4] * M_PI / 180 - pose[2];
    lastIn[4] = in[4];
  }else{
    dTheta = (dR - dL) / (2 * vals.rlTrackingWidth) +
             (dF - dB) / (2 * vals.fbTrackingWidth);
  }

  double dV = (dR + dL) / 2.0;
  double dSV = (dF + dB) / 2.0;

  double lsin = sin((pose[2]) + 0.5 * dTheta);
  double lcos = cos((pose[2]) + 0.5 * dTheta);

  pose[0] += lcos * dV -
             lsin * dSV;
  pose[1] += lcos * dSV +
             lsin * dV;
  pose[2] += dTheta;

  pose[3] = velFilter->   filter(dV  / dT);
  pose[4] = stfVelFilter->filter(dSV / dT);
  pose[5] = angVelFilter->filter(dTheta / dT);

  return pose;
}

const std::array<double,6> &Odom4EncImuSimp::get() {
  return pose;
}
