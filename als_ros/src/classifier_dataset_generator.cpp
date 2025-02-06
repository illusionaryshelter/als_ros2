#include <als_ros/classifier_dataset_generator.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto generator = std::make_shared<als_ros::ClassifierDatasetGenerator>();
  generator->generateDataset();
  return 0;
}
