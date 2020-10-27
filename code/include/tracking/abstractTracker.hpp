#pragma once
#include "kappa/api.hpp"

template <std::size_t N>
class AbstractTracker : public kappa::AbstractController<std::array<double,6>,std::shared_ptr<kappa::FileInput<N>>,std::tuple<double,double>> {
public:
  virtual void setTarget(const std::shared_ptr<kappa::FileInput<N>> &itarget) override {
    if(pathFile) {
      delete pathFile;
    }

    pathFile = itarget;
    this->reset();

    activeWaypoint = pathFile->get();
  }

protected:
  std::shared_ptr<kappa::FileInput<N>> pathFile{nullptr};
  std::array<double,N> activeWaypoint;
};