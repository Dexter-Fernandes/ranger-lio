// Smoke test for the toolchain: constructs one object from each dependency.
#include <cstdio>
#include <vector>

#include <Eigen/Core>
#include <gtsam/geometry/Pose3.h>
#include <small_gicp/points/point_cloud.hpp>

#include "ranger_lio/se3.hpp"

int main() {
  const gtsam::Pose3 pose;  // identity
  small_gicp::PointCloud cloud(std::vector<Eigen::Vector4d>{{1.0, 0.0, 0.0, 1.0}});
  const ranger_lio::SE3 T;  // identity
  std::printf("gtsam Pose3 ok (t=%.1f), small_gicp cloud ok (%zu pts), ranger_lio SE3 ok (%.1f)\n",
              pose.translation().x(), cloud.size(), T.translation().x());
  return 0;
}
