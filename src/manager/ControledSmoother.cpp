#include "manager/ControledSmoother.hpp"

#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/BayesTreeMarginalizationHelper.h>

namespace rplrio {

ControledSmoother::ControledSmoother(double lag,
                                     const gtsam::ISAM2Params &params)
    : IncrementalFixedLagSmoother(lag, params) {}

FixedLagSmoother::Result ControledSmoother::update(
    NonlinearFactorGraph &newFactors, const Values &newTheta,
    const KeyTimestampMap &timestamps,
    const noiseModel::Diagonal::shared_ptr &marginalizationPoseModel,
    const FactorIndices &factorsToRemove) {
  FastVector<size_t> removedFactors;
  std::optional<FastMap<Key, int>> constrainedKeys = {};

  // Update the Timestamps associated with the factor keys
  updateKeyTimestampMap(timestamps);

  // Get current timestamp
  double current_timestamp = getCurrentTimestamp();

  // Find the set of variables to be marginalized out
  KeyVector marginalizableKeys =
      findKeysBefore(current_timestamp - smootherLag_);

  // Add priors on the marginalizable pose keys to prevent uncertainty growth
  for (auto k : marginalizableKeys) {
    Symbol sym(k);
    if (sym.chr() == 'x') {
      Pose3 x_est = calculateEstimate<Pose3>(k);
      x_est =
          Pose3(x_est.rotation(), gtsam::Point3(x_est.translation().x(),
                                                x_est.translation().y(), 0));
      newFactors.addPrior(sym, x_est, marginalizationPoseModel);
    }
  }

  // Force iSAM2 to put the marginalizable variables at the beginning
  createOrderingConstraints(marginalizableKeys, constrainedKeys);

  std::unordered_set<Key> additionalKeys =
      BayesTreeMarginalizationHelper<ISAM2>::gatherAdditionalKeysToReEliminate(
          isam_, marginalizableKeys);
  KeyList additionalMarkedKeys(additionalKeys.begin(), additionalKeys.end());

  // Update iSAM2
  isamResult_ = isam_.update(newFactors, newTheta, factorsToRemove,
                             constrainedKeys, {}, additionalMarkedKeys);

  // Marginalize out any needed variables
  if (marginalizableKeys.size() > 0) {
    FastList<Key> leafKeys(marginalizableKeys.begin(),
                           marginalizableKeys.end());
    isam_.marginalizeLeaves(leafKeys);
  }

  // Remove marginalized keys from the KeyTimestampMap
  eraseKeyTimestampMap(marginalizableKeys);

  // TODO: Fill in result structure
  Result result;
  result.iterations = 1;
  result.linearVariables = 0;
  result.nonlinearVariables = 0;
  result.error = 0;

  return result;
}

void ControledSmoother::printSymbolicTree() const {
  if (!isam_.roots().empty()) {
    for (const gtsam::ISAM2::sharedClique &root : isam_.roots()) {
      printSymbolicTreeHelper(root);
    }
  } else
    std::cout << "{Empty Tree}" << std::endl;
}

void ControledSmoother::printSymbolicTreeHelper(
    const ISAM2Clique::shared_ptr &clique, const std::string indent) const {
  // Print the current clique
  std::cout << indent << "P( ";
  for (Key key : clique->conditional()->frontals()) {
    std::cout << DefaultKeyFormatter(key) << " ";
  }
  if (clique->conditional()->nrParents() > 0)
    std::cout << "| ";
  for (Key key : clique->conditional()->parents()) {
    std::cout << DefaultKeyFormatter(key) << " ";
  }
  std::cout << ")" << std::endl;

  // Recursively print all of the children
  for (const ISAM2Clique::shared_ptr &child : clique->children) {
    printSymbolicTreeHelper(child, indent + " ");
  }
}

} // namespace rplrio
