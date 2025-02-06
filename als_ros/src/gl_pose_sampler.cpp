#include <als_ros/gl_pose_sampler.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto sampler = std::make_shared<als_ros::GLPoseSampler>();
  rclcpp::spin(sampler);
  rclcpp::shutdown();
}
