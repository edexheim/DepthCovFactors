#pragma once

#include "cpp/depth_norm.h"
#include "cpp/GpDepthFactorBase.h"

namespace gtsam_gpdepth {

using namespace gtsam;

class GpDepthFactor2D : public GpDepthFactorBase {

public:
  typedef GpDepthFactorBase Base;
  typedef GpDepthFactor2D This;

public:
  typedef boost::shared_ptr<This> shared_ptr;

  /** Default constructor for I/O only */
  GpDepthFactor2D() {}

  /** Destructor */
  virtual ~GpDepthFactor2D() {}
  
  /** Constructor
   * @param L is the lower triangular matrix of the covariance 
  */
  GpDepthFactor2D(const SharedNoiseModel& noise_model, 
      const KeyVector& depth_keys, 
      const Matrix& L) :
    GpDepthFactorBase(noise_model, L)
  {
    // Allocate keys
    keys_.reserve(depth_keys.size());
    for (const auto& key : depth_keys)
      keys_.push_back(key);

    if (size() != L_.rows()) {
      throw std::runtime_error("GpDepthFactor2D: Cholesky factor L and number of keys do not match.");
    }
  }

  Vector getDataVector(const Values& x,
    boost::optional<std::vector<Matrix>&> H = boost::none) const {

    size_t n = size();
    Vector depth(n);
    for (size_t j = 0; j < n; j++) {
      double dd_dqz;
      depth[j] = depth_norm::logDepth(x.at<double>(keys_[j]), 
          H ? dd_dqz : static_cast<boost::optional<double&>>(boost::none));
      if (H) {
        Matrix J = Matrix::Zero(n, 1);
        J(j, 0) = dd_dqz;
        (*H)[j] = J;
      }
    }
    return depth;
  }

  Matrix jacobianVectorsToMatrix(const std::vector<Matrix>& H) const {
    size_t num_depth = size();
    Matrix J = Matrix::Zero(num_depth, num_depth);
    for (size_t j = 0; j < num_depth; j++) {
      J.col(j) = H[j];
    }
    return J;
  }

  void whitenedJacobianMatrixToVectors(Matrix A, std::vector<Matrix>& H) const {
    size_t num_depth = size();
    for (size_t j = 0; j < num_depth; j++) {
      H[j] = A.col(j);
    }
  }
  

};

} // namespace gtsam_gpdepth
