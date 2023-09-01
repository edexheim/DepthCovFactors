namespace gtsam_gpdepth {

#include <cpp/GpDepthFactorBase.h>
virtual class GpDepthFactorBase : gtsam::NoiseModelFactor {
  Vector getDataVector(const gtsam::Values& x);
  Vector unwhitenedError(const gtsam::Values& x);
};

#include <cpp/GpDepthFactor2D.h>
virtual class GpDepthFactor2D : gtsam_gpdepth::GpDepthFactorBase {
  GpDepthFactor2D(const gtsam::SharedNoiseModel& noise_model, const gtsam::KeyVector& keys, 
      const gtsam::Matrix& L);
  
  Vector getDataVector(const gtsam::Values& x);
  Vector unwhitenedError(const gtsam::Values& x);
};

#include <cpp/GpDepthFactor3D.h>
virtual class GpDepthFactor3D : gtsam_gpdepth::GpDepthFactorBase {
  GpDepthFactor3D(const gtsam::SharedNoiseModel& noise_model, 
      const gtsam::Key& pose_key, const gtsam::KeyVector& landmark_keys,
      const gtsam::Key& mean_key, const gtsam::Matrix& L);
  
  Vector getDepthVector(const gtsam::Values& x);
  Vector getDataVector(const gtsam::Values& x);
  Vector unwhitenedError(const gtsam::Values& x);
};

}  // namespace gtsam_gpdepth
