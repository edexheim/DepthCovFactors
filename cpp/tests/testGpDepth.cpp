#include "cpp/GpDepthFactor2D.h"
#include "cpp/GpDepthFactor3D.h"

#include <random>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace gtsam_gpdepth;

// Set to 1 to enable verbose output of intermediary results
#define TEST_VERBOSE_OUTPUT 1

#if TEST_VERBOSE_OUTPUT
#define TEST_COUT(ARGS_) std::cout << ARGS_
#else
#define TEST_COUT(ARGS_) void(0)
#endif

Matrix getRandomL(size_t n, default_random_engine& eng, uniform_real_distribution<double> distr) {
  Matrix L = Matrix::Zero(n,n);
  for (size_t i = 0; i < n; i++) {
    for (size_t j = 0; j <= i; j++) {
      double a = distr(eng);
      if (i==j) // Make diagonal positive
        a += 1.0;
        // a = abs(a);
      L(i,j) = a;
    }
  }

  return L;
}

vector<Matrix> numericalJacobian2D(const GpDepthFactor2D& factor,
    const Values& values) {

  const double delta = 1e-5;
  const double finite_diff_factor = 1.0 / (2.0 * delta);

  size_t n = factor.size();
  const KeyVector& keys = factor.keys();
  vector<Matrix> H(factor.size());

  for (size_t i = 0; i < n; i++) {
    Key key = keys[i];
    double val_orig = values.at<double>(key);

    Values vals_plus = values;
    vals_plus.update(key, val_orig + delta);
    Vector err_plus = factor.unwhitenedError(vals_plus);

    Values vals_minus = values;
    vals_minus.update(key, val_orig - delta);
    Vector err_minus = factor.unwhitenedError(vals_minus);

    H[i] = (err_plus - err_minus) * finite_diff_factor;
  }
  return H;
}

vector<Matrix> numericalJacobian3D(const GpDepthFactor3D& factor,
    const Values& values) {

  const double delta = 1e-5;
  const double finite_diff_factor = 1.0 / (2.0 * delta);

  size_t n = factor.size() - 2;
  const KeyVector& keys = factor.keys();
  vector<Matrix> H(factor.size());

  // Pose
  const Key& pose_key = keys[0];
  Pose3 pose = values.at<Pose3>(keys[pose_key]);
  traits<Pose3>::TangentVector d_pose;
  Matrix H_pose(n, 6); 
  for (int j=0; j<6; j++) {
    Values vals_plus = values;
    d_pose(j) = delta;
    vals_plus.update(pose_key, traits<Pose3>::Retract(pose, d_pose));
    Vector err_plus = factor.unwhitenedError(vals_plus);
    
    Values vals_minus = values;
    d_pose(j) = -delta;
    vals_minus.update(pose_key, traits<Pose3>::Retract(pose, d_pose));
    Vector err_minus = factor.unwhitenedError(vals_minus);

    d_pose(j) = 0;
    H_pose.col(j) = (err_plus - err_minus) * finite_diff_factor;
  }
  H[0] = H_pose;

  // Scale
  const Key& scale_key = keys[1];
  Vector1 scale = values.at<Vector1>(keys[scale_key]);
  traits<Vector1>::TangentVector d_scale;
  Matrix H_scale(n, 1); 
  for (int j=0; j<1; j++) {
    Values vals_plus = values;
    d_scale(j) = delta;
    vals_plus.update(scale_key, traits<Vector1>::Retract(scale, d_scale));
    Vector err_plus = factor.unwhitenedError(vals_plus);
    
    Values vals_minus = values;
    d_scale(j) = -delta;
    vals_minus.update(scale_key, traits<Vector1>::Retract(scale, d_scale));
    Vector err_minus = factor.unwhitenedError(vals_minus);

    d_scale(j) = 0;
    H_scale.col(j) = (err_plus - err_minus) * finite_diff_factor;
  }
  H[1] = H_scale;

  // Landmarks
   traits<Pose3>::TangentVector d;
  for (size_t i = 2; i < keys.size(); i++) {
    const Key& landmark_key = keys[i];
    Point3 landmark = values.at<Point3>(landmark_key);
    traits<Point3>::TangentVector d;
    Matrix H_landmark(n, 3); 
    for (int j=0; j<3; j++) {
      Values vals_plus = values;
      d(j) = delta;
      vals_plus.update(landmark_key, traits<Point3>::Retract(landmark, d));
      Vector err_plus = factor.unwhitenedError(vals_plus);
      
      Values vals_minus = values;
      d(j) = -delta;
      vals_minus.update(landmark_key, traits<Point3>::Retract(landmark, d));
      Vector err_minus = factor.unwhitenedError(vals_minus);

      d(j) = 0;
      H_landmark.col(j) = (err_plus - err_minus) * finite_diff_factor;
    }
    H[i] = H_landmark;
  }

  return H;
}

/* ************************************************************************* */
// TEST( GpDepthFactor2D, jacobians )
// {
//   // Random
//   default_random_engine eng(1);
//   uniform_real_distribution<double> distr(0.1, 5.0);
  
//   size_t n = 8;
//   double mean = 7.0;
//   Matrix L = getRandomL(n, eng, distr);

//   Values values;
//   for (size_t i = 0; i < n; i++) {
//     values.insert(i+1, distr(eng));
//   }

//   auto noise_model = noiseModel::Isotropic::Sigma(n, 1.0);
//   GpDepthFactor2D factor(noise_model, values.keys(), L, mean);

//   // Analytic Jacobians
//   vector<Matrix> H_a(factor.size());
//   Vector err_a = factor.unwhitenedError(values, H_a);
  
//   // Numerical Jacobians
//   vector<Matrix> H_n = numericalJacobian2D(factor, values);

//   // Test
//   for (size_t i = 0; i < n; i++) {
//     EXPECT(assert_equal(H_n[i], H_a[i], 1e-4));
//   }
// }

/* ************************************************************************* */
TEST( GpDepthFactor3D, jacobians )
{
  // Random
  default_random_engine eng(1);
  uniform_real_distribution<double> distr(0.1, 5.0);
  
  size_t n = 8;
  double mean = 1.0;
  Matrix L = getRandomL(n, eng, distr);

  // Values
  Values values;
  values.insert(0, Pose3()); // pose
  values.insert(1, Vector1(mean)); // scale
  Values landmark_values;
  for (size_t i = 0; i < n; i++) {
    Point3 landmark = Point3(distr(eng), distr(eng), distr(eng));
    landmark_values.insert(i+2, landmark);
  }
  values.insert(landmark_values);

  auto noise_model = noiseModel::Isotropic::Sigma(n, 1.0);
  GpDepthFactor3D factor(noise_model, 0, landmark_values.keys(), 1, L);

  // Analytic Jacobians
  vector<Matrix> H_a(factor.size());
  Vector err_a = factor.unwhitenedError(values, H_a);

  // Numerical Jacobians
  vector<Matrix> H_n = numericalJacobian3D(factor, values);

  // Test
  for (size_t i = 0; i < H_a.size(); i++) {
    EXPECT(assert_equal(H_n[i], H_a[i], 1e-4));
  }
}

/* ************************************************************************* */
// TEST( GpDepthFactor2D, error )
// {
//   // Random
//   default_random_engine eng(1);
//   uniform_real_distribution<double> distr(0.1, 5.0);
  
//   size_t n = 8;
//   double mean = 7.0;
//   Matrix L = getRandomL(n, eng, distr);

//   Values values;
//   for (size_t i = 0; i < n; i++) {
//     values.insert(i+1, distr(eng));
//   }

//   auto noise_model = noiseModel::Isotropic::Sigma(n, 1.0);
//   GpDepthFactor2D factor(noise_model, values.keys(), L, mean);

//   double err_u = noise_model->loss(noise_model->squaredMahalanobisDistance(factor.unwhitenedError(values)));
//   std::cout << err_u << std::endl;
//   double err = factor.error(values);
//   EXPECT(assert_equal(err_u, err, 1e-4));
// }

/* ************************************************************************* */
// TEST( GpDepthFactor3D, error )
// {
//   // Random
//   default_random_engine eng(1);
//   uniform_real_distribution<double> distr(0.1, 5.0);
  
//   size_t n = 8;
//   double mean = 7.0;
//   Matrix L = getRandomL(n, eng, distr);

//   // Values
//   Values values;
//   values.insert(0, Pose3());
//   Values landmark_values;
//   for (size_t i = 0; i < n; i++) {
//     Point3 landmark = Point3(distr(eng), distr(eng), distr(eng));
//     landmark_values.insert(i+1, landmark);
//   }
//   values.insert(landmark_values);

//   auto noise_model = noiseModel::Diagonal::Sigmas(Vector1::Constant(1.0));
//   GpDepthFactor3D factor(noise_model, 0, landmark_values.keys(), L, mean);

//   double err_u = noise_model->loss(noise_model->squaredMahalanobisDistance(factor.unwhitenedError(values)));
//   std::cout << err_u << std::endl;
//   double err = factor.error(values);
//   EXPECT(assert_equal(err_u, err, 1e-4));
// }

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */