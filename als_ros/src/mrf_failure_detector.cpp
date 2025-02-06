#include <als_ros/mrf_failure_detector.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto detector = std::make_shared<als_ros::MRFFD>();
  double failureDetectionHz = detector->getFailureDetectionHz();
  rclcpp::Rate loopRate(failureDetectionHz);

  while (rclcpp::ok()) {
    rclcpp::spin_some(detector);
    detector->setCanUpdateResidualErrors(false);
    detector->predictFailureProbability();
    detector->publishROSMessages();
    detector->setCanUpdateResidualErrors(true);
    detector->printFailureProbability();
    loopRate.sleep();
  }

  return 0;
}
