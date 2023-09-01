#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>

#include <iostream>
#include <string>
#include <vector>

#include "cpp/depth_norm.h"

namespace gtsam_gpdepth {

using namespace gtsam;

class GpDepthFactorBase : public NoiseModelFactor {

public:
  typedef NoiseModelFactor Base;
  typedef GpDepthFactorBase This;

protected:
  Matrix L_;

public:
  typedef boost::shared_ptr<This> shared_ptr;

  /** Default constructor for I/O only */
  GpDepthFactorBase() {}

  /** Destructor */
  virtual ~GpDepthFactorBase() {}
  
  /** Constructor
   * @param L is the lower triangular matrix of the covariance 
  */
  GpDepthFactorBase(const SharedNoiseModel& noise_model, 
      const Matrix& L) :
    Base(noise_model), L_(L)
  {  }

  virtual Vector getDataVector(const Values& x,
    boost::optional<std::vector<Matrix>&> H = boost::none) const = 0;

  virtual Matrix jacobianVectorsToMatrix(const std::vector<Matrix>& H) const = 0;

  virtual void whitenedJacobianMatrixToVectors(Matrix A, std::vector<Matrix>& H) const = 0;

  // Technically this error is whitened?
  // How should we handle noise model? standard deviation should always be 1...
  Vector unwhitenedError(const Values& x,
      boost::optional<std::vector<Matrix>&> H = boost::none) const override {

    Vector depth;
    if (H)
      depth = getDataVector(x, H);
    else 
      depth = getDataVector(x);
    
    // Error
    Vector centered_err = depth.array();
    Vector err_vec = L_.triangularView<Eigen::Lower>().solve(centered_err);

    // Linear Jacobian
    if (H) {
      Matrix J = jacobianVectorsToMatrix(*H);
      Matrix A = L_.triangularView<Eigen::Lower>().solve(J);
      whitenedJacobianMatrixToVectors(A, *H);
    }

    return err_vec;
  }
  

};

} // namespace gtsam_gpdepth
