#include <als_ros/classifier_dataset_generator.hpp>
#include <als_ros/mae_classifier.hpp>
#include <als_ros/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::Node nh("nh");
  std::vector<std::string> trainDirs, testDirs;
  std::string classifierDir;
  double maxResidualError = 0, maeHistogramBinWidth = 0;

  nh.declare_parameter("train_dirs", trainDirs);
  nh.declare_parameter("test_dirs", testDirs);
  nh.declare_parameter("classifier_dir", classifierDir);
  nh.declare_parameter("max_residual_error", maxResidualError);
  nh.declare_parameter("histogram_bin_width", maeHistogramBinWidth);

  nh.get_parameter("train_dirs", trainDirs);
  nh.get_parameter("test_dirs", testDirs);
  nh.get_parameter("classifier_dir", classifierDir);
  nh.get_parameter("max_residual_error", maxResidualError);
  nh.get_parameter("histogram_bin_width", maeHistogramBinWidth);

#ifdef GET_PARAM_DEBUG
  RCLCPP_INFO_STREAM(nh.get_logger(),
                     "max_residual_error:" << maxResidualError);
  RCLCPP_INFO_STREAM(nh.get_logger(),
                     "histogram_bin_width:" << maeHistogramBinWidth);
  RCLCPP_INFO_STREAM(nh.get_logger(), "classifierDir:" << classifierDir);
  RCLCPP_INFO_STREAM(nh.get_logger(), "test_dirs:" << testDirs);
  RCLCPP_INFO_STREAM(nh.get_logger(), "train_dirs:" << trainDirs);
#endif
  exit(0);

  std::vector<als_ros::Pose> gtPosesTrain, successPosesTrain, failurePosesTrain;
  std::vector<sensor_msgs::msg::LaserScan> scansTrain;
  std::vector<std::vector<double>> successResidualErrorsTrain,
      failureResidualErrorsTrain;

  std::vector<als_ros::Pose> gtPosesTest, successPosesTest, failurePosesTest;
  std::vector<sensor_msgs::msg::LaserScan> scansTest;
  std::vector<std::vector<double>> successResidualErrorsTest,
      failureResidualErrorsTest;

  auto generator = std::make_shared<als_ros::ClassifierDatasetGenerator>();
  generator->setTrainDirs(trainDirs);
  generator->setTestDirs(testDirs);
  generator->readTrainDataset(
      gtPosesTrain, successPosesTrain, failurePosesTrain, scansTrain,
      successResidualErrorsTrain, failureResidualErrorsTrain);
  generator->readTestDataset(gtPosesTest, successPosesTest, failurePosesTest,
                             scansTest, successResidualErrorsTest,
                             failureResidualErrorsTest);

  auto classifier = std::make_shared<als_ros::MAEClassifier>();
  classifier->setClassifierDir(classifierDir);
  classifier->setMaxResidualError(maxResidualError);
  classifier->setMAEHistogramBinWidth(maeHistogramBinWidth);
  classifier->learnThreshold(successResidualErrorsTrain,
                             failureResidualErrorsTrain);
  classifier->writeClassifierParams(successResidualErrorsTest,
                                    failureResidualErrorsTest);
  classifier->writeDecisionLikelihoods();

  return 0;
}
