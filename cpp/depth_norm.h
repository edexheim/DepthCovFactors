#pragma once

#include <boost/optional.hpp>
#include <cmath>

namespace gtsam_gpdepth {

namespace depth_norm {
  
double invDepth(double z, 
    boost::optional<double&> H = boost::none) {
  double d = 1.0/z;
  if (H) {
    (*H) = -d*d;
  }
  return d;
}

double logDepth(double z, 
    boost::optional<double&> H = boost::none) {
  double d = std::log(z);
  if (H) {
    (*H) = 1.0/z;
  }
  return d;
}

} // namespace depth_norm

} // namespace gtsam_gpdepth