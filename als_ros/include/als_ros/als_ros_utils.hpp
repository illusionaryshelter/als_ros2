#pragma once
#include <iostream>
#include <vector>

template <typename T>
std::ostream &operator<<(std::ostream &os, std::vector<T> &__vec) {
  // os << "size:" << __vec.size();
  os << '[';

  for (auto &i : __vec)
    os << i << ' ';

  os << ']';
  return os;
}

#ifdef ALS_PARAM_DEBUG
#define GET_PARAM_DEBUG(name, param)                                           \
  this->get_parameter(name, param);                                            \
  RCLCPP_INFO_STREAM(this->get_logger(), name << ":\n" << param);
#else
#define GET_PARAM_DEBUG(name, param) this->get_parameter(name, param);
#endif

/**
./include/als_ros/gl_pose_sampler.hpp:  rclcpp::Publisher<PoseArrayT>::SharedPtr
posesPub_;
./include/als_ros/gl_pose_sampler.hpp:  rclcpp::Publisher<MapT>::SharedPtr
localMapPub_;
./include/als_ros/gl_pose_sampler.hpp:  rclcpp::Publisher<MarkerT>::SharedPtr
sdfKeypointsPub_;
./include/als_ros/gl_pose_sampler.hpp:  rclcpp::Publisher<MarkerT>::SharedPtr
localSDFKeypointsPub_;
./include/als_ros/mcl.hpp:  rclcpp::Publisher<PoseStampT>::SharedPtr posePub_;
./include/als_ros/mcl.hpp:  rclcpp::Publisher<PoseArrayT>::SharedPtr
particlesPub_;
./include/als_ros/mcl.hpp:  rclcpp::Publisher<LaserT>::SharedPtr
unknownScanPub_;
./include/als_ros/mcl.hpp:  rclcpp::Publisher<LaserT>::SharedPtr
residualErrorsPub_;
./include/als_ros/mcl.hpp:  rclcpp::Publisher<ReliabilityT>::SharedPtr
reliabilityPub_;
./include/als_ros/mcl.hpp:  rclcpp::Publisher<MarkerT>::SharedPtr
reliabilityMarkerPub_;
./include/als_ros/mrf_failure_detector.hpp:  rclcpp::Publisher<ProbT>::SharedPtr
failureProbPub_;
./include/als_ros/mrf_failure_detector.hpp: rclcpp::Publisher<LaserT>::SharedPtr
alignedScanPub_;
./include/als_ros/mrf_failure_detector.hpp: rclcpp::Publisher<LaserT>::SharedPtr
misalignedScanPub_;
./include/als_ros/mrf_failure_detector.hpp: rclcpp::Publisher<LaserT>::SharedPtr
unknownScanPub_;
./include/als_ros/mrf_failure_detector.hpp:
rclcpp::Publisher<MarkerT>::SharedPtr failureProbabilityMarkerPub_;
./src/scan2pc.cpp:  rclcpp::Publisher<PointCloudT>::SharedPtr pcPub_;
 */