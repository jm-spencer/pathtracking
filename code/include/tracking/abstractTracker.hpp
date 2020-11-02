#pragma once
#include "kappa/api.hpp"

template <std::size_t N>
class AbstractTracker : public kappa::AbstractController<std::array<double,6>,std::shared_ptr<kappa::AbstractInput<std::array<double,N>>>,std::tuple<double,double>> {
public:
  virtual void setTarget(const std::shared_ptr<kappa::AbstractInput<std::array<double,N>>> &itarget) override {
    this->reset();
    pathFile = itarget;

    activeWaypoint = pathFile->get();
  }

protected:
  std::shared_ptr<kappa::AbstractInput<std::array<double,N>>> pathFile{nullptr};
  std::array<double,N> activeWaypoint;
};
