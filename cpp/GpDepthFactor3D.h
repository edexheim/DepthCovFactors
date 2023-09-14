#pragma once

#include <cmath>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

#include "cpp/depth_norm.h"
#include "cpp/GpDepthFactorBase.h"

namespace gtsam_gpdepth {

using namespace gtsam;

class GpDepthFactor3D : public GpDepthFactorBase {

public:
  typedef NoiseModelFactor Base;
  typedef GpDepthFactor3D This;

public:
  typedef boost::shared_ptr<This> shared_ptr;

  /** Default constructor for I/O only */
  GpDepthFactor3D() {}

  /** Destructor */
  virtual ~GpDepthFactor3D() {}
  
  /** Constructor
   * @param L is the lower triangular matrix of the covariance 
  */
  GpDepthFactor3D(const SharedNoiseModel& noise_model, 
      const Key& pose_key, const KeyVector& landmark_keys, const Key& mean_key,
      const Matrix& L) :
    GpDepthFactorBase(noise_model, L)
  {
    // Allocate keys with pose first
    keys_.reserve(2+landmark_keys.size());
    keys_.push_back(pose_key);
    keys_.push_back(mean_key);
    for (const auto& key : landmark_keys) {
      keys_.push_back(key);
    }

    // One key is pose key, rest are landmarks
    if (size()-2 != L_.rows()) {
      throw std::runtime_error("GpDepthFactor3D: Cholesky factor L and number of keys do not match.");
    }
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  Vector getDepthVector(const Values& x) const {
    
    const Pose3& pose = x.at<Pose3>(keys_[0]); // T_WC

    size_t n = size();
    Vector depth(n-2);
    // Skip pose key and mean key
    for (size_t k_ind = 2; k_ind < n; k_ind++) {
      size_t z_ind = k_ind-2;
      Point3 p = x.at<Point3>(keys_[k_ind]);
      // Transform to camera coordinates
      const Point3 q = pose.transformTo(p);
      // Get inverse depth
      depth[z_ind] = q.z();
    }

    return depth;
  }

  Vector getDataVector(const Values& x, 
      boost::optional<std::vector<Matrix>&> H = boost::none) const {
    
    size_t num_depth = size() - 2;

    // Setup for pose variable
    const Pose3& pose = x.at<Pose3>(keys_[0]); // T_WC
    if (H) {
      (*H)[0] = Matrix(num_depth, 6);
    }

    const Vector1& mean_vec = x.at<Vector1>(keys_[1]);
    double mean = mean_vec(0);
    if (H) {
      (*H)[1] = -Matrix::Ones(num_depth, 1);
    }

    double eps = 1e-6;

    size_t n = size();
    Vector data_vector(n-2);
    // Skip pose key and mean key
    for (size_t k_ind = 2; k_ind < n; k_ind++) {
      size_t z_ind = k_ind-2;
      Point3 p = x.at<Point3>(keys_[k_ind]);
      // Transform to camera coordinates
      Matrix36 F;
      Matrix33 E;
      const Point3 q = pose.transformTo(p, H ? &F : 0, H ? &E : 0);
      // Get log depth
      bool valid_log_depth = q.z() > eps;
      double dd_dqz;
      if (valid_log_depth) {
        data_vector[z_ind] = depth_norm::logDepth(q.z(), H ? dd_dqz : static_cast<boost::optional<double&>>(boost::none));
        data_vector[z_ind] -= mean;
      }
      else{
        // Use linear model if log is invalid (point is behind camera)
        data_vector[z_ind] = q.z() + std::log(eps) - eps - mean;
        // std::cout << keys_[k_ind] << " " << q.z() << std::endl;
      }
      // Jacobians
      if (H) {
        if (valid_log_depth) {
          // Landmark
          Matrix dqz_dp = E.row(2);
          (*H)[k_ind] = Matrix::Zero(num_depth, 3);
          // Only z part of landmark will have non-zero Jacobian 
          (*H)[k_ind].row(z_ind) = dd_dqz * dqz_dp;
          // Pose
          (*H)[0].row(z_ind) = dd_dqz * F.row(2);
        }
        else {
          Matrix dqz_dp = E.row(2);
          (*H)[k_ind] = Matrix::Zero(num_depth, 3);
          double dd_dqz = 1.0;
          (*H)[k_ind].row(z_ind) = dd_dqz * dqz_dp;
          // (*H)[0].row(z_ind) = Matrix::Zero(1, 6);
          // (*H)[1].row(z_ind) = Matrix::Zero(1, 1);
          // Pose
          (*H)[0].row(z_ind) = dd_dqz * F.row(2);
        }
      }
    }

    return data_vector;
  }

  Matrix jacobianVectorsToMatrix(const std::vector<Matrix>& H) const {
    size_t num_depth = size() - 2;
    Matrix J = Matrix::Zero(num_depth, 7 + 3*num_depth);
    J.block(0,0,num_depth,6) = H[0];
    J.block(0,6,num_depth,1) = H[1];
    for (size_t j = 0; j < num_depth; j++) {
      J.block(0, 7+3*j, num_depth, 3) = H[j+2];
    }
    return J;
  }

  void whitenedJacobianMatrixToVectors(Matrix A, std::vector<Matrix>& H) const {
    size_t num_depth = size() - 2;
    H[0] = A.block(0,0,num_depth,6);
    H[1] = A.block(0,6,num_depth,1);
    for (size_t j = 0; j < num_depth; j++) {
      H[j+2] = A.block(0, 7+3*j, num_depth, 3);
    }
  }

};

} // namespace gtsam_gpdepth
