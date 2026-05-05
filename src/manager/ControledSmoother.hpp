#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <memory>
#include <unordered_set>

using namespace gtsam;

namespace rplrio {

class ControledSmoother : public gtsam::IncrementalFixedLagSmoother {
public:
  ControledSmoother(double lag = 0.0,
                    const gtsam::ISAM2Params &params = gtsam::ISAM2Params(),
                    bool two_d_only = false);

  virtual ~ControledSmoother() = default;

  FixedLagSmoother::Result
  update(NonlinearFactorGraph &newFactors, const Values &newTheta,
         const KeyTimestampMap &timestamps,
         const noiseModel::Diagonal::shared_ptr &marginalizationPoseModel,
         const FactorIndices &factorsToRemove = FactorIndices());

  void printSymbolicTree() const;
  void printSymbolicTreeHelper(const ISAM2Clique::shared_ptr &clique,
                               const std::string indent = "") const;

private:
  bool two_d_only_ = false;
};

} // namespace rplrio
