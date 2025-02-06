#include <als_ros/mcl.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto mcl = std::make_shared<als_ros::MCL>();

  double localizationHz = mcl->getLocalizationHz();
  rclcpp::Rate loopRate(localizationHz);

  while (rclcpp::ok()) {
    rclcpp::spin_some(mcl);
    mcl->updateParticlesByMotionModel();
    mcl->setCanUpdateScan(false);
    mcl->calculateLikelihoodsByMeasurementModel();
    mcl->calculateLikelihoodsByDecisionModel();
    mcl->calculateGLSampledPosesLikelihood();
    mcl->calculateAMCLRandomParticlesRate();
    mcl->calculateEffectiveSampleSize();
    mcl->estimatePose();
    mcl->resampleParticles();
    // mcl->plotScan();
    // mcl->plotWorld(50->0);
    mcl->publishROSMessages();
    mcl->broadcastTF();
    // mcl->plotLikelihoodMap();
    mcl->setCanUpdateScan(true);
    mcl->printResult();
    loopRate.sleep();
  }

  rclcpp::shutdown();
}