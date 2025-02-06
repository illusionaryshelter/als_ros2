#pragma once
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include <als_ros/als_ros_utils.hpp>

namespace als_ros {
enum Measurement { ALIGNED, MISALIGNED, UNKNOWN };

class MRFFD : public rclcpp::Node {

public:
  using LaserT = sensor_msgs::msg::LaserScan;
  using MarkerT = visualization_msgs::msg::Marker;
  using ProbT = geometry_msgs::msg::Vector3Stamped;

private:
  // pub\sub
  std::string residualErrorsName_;
  rclcpp::Subscription<LaserT>::SharedPtr residualErrorsSub_;

  std::string failureProbName_, alignedScanName_, misalignedScanName_,
      unknownScanName_;
  rclcpp::Publisher<ProbT>::SharedPtr failureProbPub_;
  rclcpp::Publisher<LaserT>::SharedPtr alignedScanPub_;
  rclcpp::Publisher<LaserT>::SharedPtr misalignedScanPub_;
  rclcpp::Publisher<LaserT>::SharedPtr unknownScanPub_;
  bool publishClassifiedScans_ = true;

  std::string failureProbabilityMarkerName_, markerFrame_;
  bool publishFailureProbabilityMarker_ = true;
  rclcpp::Publisher<MarkerT>::SharedPtr failureProbabilityMarkerPub_;

  // param
  double maxResidualError_ = 1.0;
  double NDMean_ = 0., NDVar_ = 0.04, EDLambda_ = 4., NDNormConst_;
  int minValidResidualErrorsNum_ = 10, maxResidualErrorsNum_ = 200;
  int maxLPBComputationNum_ = 1000;
  int samplingNum_ = 1000;
  double residualErrorReso_ = 0.05;
  double misalignmentRatioThreshold_ = 0.1, unknownRatioThreshold_ = 0.7;
  std::vector<double> transitionProbMat_;

  LaserT residualErrors_;
  std::vector<double> usedResidualErrors_;
  std::vector<int> usedScanIndices_;
  bool canUpdateResidualErrors_ = true, gotResidualErrors_ = false;
  double failureDetectionHz_ = 10.;

  // result
  std::vector<std::vector<double>> measurementClassProbabilities_;
  double failureProbability_;
  rclcpp::Time failureProbabilityStamp_;

public:
  MRFFD()
      : Node("MRFFD"), residualErrorsName_("/residual_errors"),
        failureProbName_("/failure_probability"),
        alignedScanName_("/aligned_scan_mrf"),
        misalignedScanName_("/misaligned_scan_mrf"),
        unknownScanName_("/unknown_scan_mrf"), publishClassifiedScans_(true),
        failureProbabilityMarkerName_("/failure_probability_marker"),
        markerFrame_("base_link"),
        transitionProbMat_(
            {0.8, 0.0, 0.2, 0.0, 0.8, 0.2, 0.333333, 0.333333, 0.333333}) {
    // in/output
    this->declare_parameter("residual_errors_name", residualErrorsName_);
    this->declare_parameter("failure_probability_name", failureProbName_);
    this->declare_parameter("publish_classified_scans",
                            publishClassifiedScans_);
    this->declare_parameter("aligned_scan_mrf", alignedScanName_);
    this->declare_parameter("misaligned_scan_mrf", misalignedScanName_);
    this->declare_parameter("unknown_scan_mrf", unknownScanName_);
    this->declare_parameter("failure_probability_marker_name",
                            failureProbabilityMarkerName_);
    this->declare_parameter("publish_failure_probability_marker",
                            publishFailureProbabilityMarker_);
    this->declare_parameter("marker_frame", markerFrame_);
    // parameters
    this->declare_parameter("normal_distribution_mean", NDMean_);
    this->declare_parameter("normal_distribution_var", NDVar_);
    this->declare_parameter("exponential_distribution_lambda", EDLambda_);
    this->declare_parameter("max_residual_error", maxResidualError_);
    this->declare_parameter("residual_error_resolution", residualErrorReso_);
    this->declare_parameter("min_valid_residual_errors_num",
                            minValidResidualErrorsNum_);
    this->declare_parameter("max_residual_errors_num", maxResidualErrorsNum_);
    this->declare_parameter("max_lpb_computation_num", maxLPBComputationNum_);
    this->declare_parameter("sampling_num", samplingNum_);
    this->declare_parameter("misalignment_ratio_threshold",
                            misalignmentRatioThreshold_);
    this->declare_parameter("unknown_ratio_threshold", unknownRatioThreshold_);
    this->declare_parameter("transition_probability_matrix",
                            transitionProbMat_);
    // other parameters
    this->declare_parameter("failure_detection_hz", failureDetectionHz_);

    // in/output
    GET_PARAM_DEBUG("residual_errors_name", residualErrorsName_);
    GET_PARAM_DEBUG("failure_probability_name", failureProbName_);
    GET_PARAM_DEBUG("publish_classified_scans",
                            publishClassifiedScans_);
    GET_PARAM_DEBUG("aligned_scan_mrf", alignedScanName_);
    GET_PARAM_DEBUG("misaligned_scan_mrf", misalignedScanName_);
    GET_PARAM_DEBUG("unknown_scan_mrf", unknownScanName_);
    GET_PARAM_DEBUG("failure_probability_marker_name",
                            failureProbabilityMarkerName_);
    GET_PARAM_DEBUG("publish_failure_probability_marker",
                            publishFailureProbabilityMarker_);
    GET_PARAM_DEBUG("marker_frame", markerFrame_);
    // parameters
    GET_PARAM_DEBUG("normal_distribution_mean", NDMean_);
    GET_PARAM_DEBUG("normal_distribution_var", NDVar_);
    GET_PARAM_DEBUG("exponential_distribution_lambda", EDLambda_);
    GET_PARAM_DEBUG("max_residual_error", maxResidualError_);
    GET_PARAM_DEBUG("residual_error_resolution", residualErrorReso_);
    GET_PARAM_DEBUG("min_valid_residual_errors_num",
                            minValidResidualErrorsNum_);
    GET_PARAM_DEBUG("max_residual_errors_num", maxResidualErrorsNum_);
    GET_PARAM_DEBUG("max_lpb_computation_num", maxLPBComputationNum_);
    GET_PARAM_DEBUG("sampling_num", samplingNum_);
    GET_PARAM_DEBUG("misalignment_ratio_threshold",
                            misalignmentRatioThreshold_);
    GET_PARAM_DEBUG("unknown_ratio_threshold", unknownRatioThreshold_);
    GET_PARAM_DEBUG("transition_probability_matrix",
                            transitionProbMat_);
    // other parameters
    GET_PARAM_DEBUG("failure_detection_hz", failureDetectionHz_);

    NDNormConst_ = 1.0 / sqrt(2.0 * M_PI * NDVar_);

    residualErrorsSub_ = rclcpp::create_subscription<LaserT>(
        *this, residualErrorsName_, 1,
        std::bind(&MRFFD::residualErrors_callback, this,
                  std::placeholders::_1));

    failureProbPub_ =
        rclcpp::create_publisher<ProbT>(*this, failureProbName_, 1);

    if (publishClassifiedScans_) {
      alignedScanPub_ =
          rclcpp::create_publisher<LaserT>(*this, alignedScanName_, 1);
      misalignedScanPub_ =
          rclcpp::create_publisher<LaserT>(*this, misalignedScanName_, 1);
      unknownScanPub_ =
          rclcpp::create_publisher<LaserT>(*this, unknownScanName_, 1);
    }

    if (publishFailureProbabilityMarker_) {
      failureProbabilityMarkerPub_ = rclcpp::create_publisher<MarkerT>(
          *this, failureProbabilityMarkerName_, 1);
    }

    rclcpp::Rate loopRate(failureDetectionHz_);
    int residualErrorsFailedCnt = 0;
    while (!gotResidualErrors_) {
      rclcpp::spin_some(this->get_node_base_interface());
      residualErrorsFailedCnt++;
      if (residualErrorsFailedCnt >= 30) {
        RCLCPP_ERROR(this->get_logger(),
                     "Cannot get residual errors."
                     " Did you publish the residual errors?"
                     " The expected topic name is %s",
                     residualErrorsName_.c_str());
      }
      loopRate.sleep();
    }
    RCLCPP_INFO(this->get_logger(),
                "MRF failure detector is ready to perform.");
  }

  void spin_some() { rclcpp::spin_some(this->get_node_base_interface()); }

  ~MRFFD() {}

  inline void setMaxResidualError(double maxResidualError) {
    maxResidualError_ = maxResidualError;
  }
  inline void setNDMean(double NDMean) { NDMean_ = NDMean; }
  inline void setNDVariance(double NDVar) {
    NDVar_ = NDVar, NDNormConst_ = 1.0 / sqrt(2.0 * M_PI * NDVar_);
  }
  inline void setEDLambda(double EDLambda) { EDLambda_ = EDLambda; }
  inline void setResidualErrorReso(double residualErrorReso) {
    residualErrorReso_ = residualErrorReso;
  }
  inline void setMinValidResidualErrorNum(int minValidResidualErrorNum) {
    minValidResidualErrorsNum_ = minValidResidualErrorNum;
  }
  inline void setMaxLPBComputationNum(int maxLPBComputationNum) {
    maxLPBComputationNum_ = maxLPBComputationNum;
  }
  inline void setSamplingNum(int samplingNum) { samplingNum_ = samplingNum; }
  inline void setMisalignmentRatioThreshold(double misalignmentRatioThreshold) {
    misalignmentRatioThreshold_ = misalignmentRatioThreshold;
  }
  inline void setTransitionProbMat(std::vector<double> transitionProbMat) {
    transitionProbMat_ = transitionProbMat;
  }
  inline void setCanUpdateResidualErrors(bool canUpdateResidualErrors) {
    canUpdateResidualErrors_ = canUpdateResidualErrors;
  }

  inline double getFailureProbability(void) { return failureProbability_; }
  inline double getMeasurementClassProbabilities(int errorIndex,
                                                 int measurementClass) {
    return measurementClassProbabilities_[errorIndex][measurementClass];
  }
  inline std::vector<double> getMeasurementClassProbabilities(int errorIndex) {
    return measurementClassProbabilities_[errorIndex];
  }
  inline double getFailureDetectionHz(void) { return failureDetectionHz_; }

  void predictFailureProbability(void) {
    std::vector<double> validResidualErrors;
    std::vector<int> validScanIndices;
    for (int i = 0; i < (int)residualErrors_.intensities.size(); ++i) {
      double e = residualErrors_.intensities[i];
      if (0.0 <= e && e <= maxResidualError_) {
        validResidualErrors.push_back(e);
        validScanIndices.push_back(i);
      }
    }

    int validResidualErrorsSize = (int)validResidualErrors.size();
    if (validResidualErrorsSize <= minValidResidualErrorsNum_) {
      RCLCPP_ERROR_STREAM(
          this->get_logger(),
          "WARNING: Number of validResidualErrors is less than the "
          "expected threshold number."
              << " The threshold is " << minValidResidualErrorsNum_
              << ", but the number of validResidualErrors "
              << validResidualErrorsSize << ".");
      failureProbability_ = -1.0;
      return;
    } else if (validResidualErrorsSize <= maxResidualErrorsNum_) {
      usedResidualErrors_ = validResidualErrors;
      usedScanIndices_ = validScanIndices;
    } else {
      usedResidualErrors_.resize(maxResidualErrorsNum_);
      usedScanIndices_.resize(maxResidualErrorsNum_);
      for (int i = 0; i < maxResidualErrorsNum_; ++i) {
        int idx = rand() % (int)validResidualErrors.size();
        usedResidualErrors_[i] = validResidualErrors[idx];
        usedScanIndices_[i] = validScanIndices[idx];
        validResidualErrors.erase(validResidualErrors.begin() + idx);
        validScanIndices.erase(validScanIndices.begin() + idx);
      }
    }

    std::vector<std::vector<double>> likelihoodVectors =
        getLikelihoodVectors(usedResidualErrors_);
    std::vector<std::vector<double>> measurementClassProbabilities =
        estimateMeasurementClassProbabilities(likelihoodVectors);
    setAllMeasurementClassProbabilities(usedResidualErrors_,
                                        measurementClassProbabilities);
    failureProbability_ =
        predictFailureProbabilityBySampling(measurementClassProbabilities);
  }

  void publishROSMessages(void) {
    ProbT failureProbability;
    failureProbability.header.stamp = residualErrors_.header.stamp;
    failureProbability.vector.x = failureProbability_;
    failureProbPub_->publish(failureProbability);

    if (publishClassifiedScans_) {
      std::vector<int> residualErrorClasses = getResidualErrorClasses();
      LaserT alignedScan, misalignedScan, unknownScan;
      alignedScan.header = misalignedScan.header = unknownScan.header =
          residualErrors_.header;
      alignedScan.range_min = misalignedScan.range_min = unknownScan.range_min =
          residualErrors_.range_min;
      alignedScan.range_max = misalignedScan.range_max = unknownScan.range_max =
          residualErrors_.range_max;
      alignedScan.angle_min = misalignedScan.angle_min = unknownScan.angle_min =
          residualErrors_.angle_min;
      alignedScan.angle_max = misalignedScan.angle_max = unknownScan.angle_max =
          residualErrors_.angle_max;
      alignedScan.angle_increment = misalignedScan.angle_increment =
          unknownScan.angle_increment = residualErrors_.angle_increment;
      alignedScan.time_increment = misalignedScan.time_increment =
          unknownScan.time_increment = residualErrors_.time_increment;
      alignedScan.scan_time = misalignedScan.scan_time = unknownScan.scan_time =
          residualErrors_.scan_time;
      int size = (int)residualErrors_.ranges.size();
      alignedScan.ranges.resize(size);
      misalignedScan.ranges.resize(size);
      unknownScan.ranges.resize(size);
      alignedScan.intensities.resize(size);
      misalignedScan.intensities.resize(size);
      unknownScan.intensities.resize(size);
      for (int i = 0; i < (int)usedResidualErrors_.size(); ++i) {
        int idx = usedScanIndices_[i];
        if (residualErrorClasses[i] == ALIGNED)
          alignedScan.ranges[idx] = residualErrors_.ranges[idx];
        else if (residualErrorClasses[i] == MISALIGNED)
          misalignedScan.ranges[idx] = residualErrors_.ranges[idx];
        else
          unknownScan.ranges[idx] = residualErrors_.ranges[idx];
      }
      alignedScanPub_->publish(alignedScan);
      misalignedScanPub_->publish(misalignedScan);
      unknownScanPub_->publish(unknownScan);
    }

    if (publishFailureProbabilityMarker_) {
      MarkerT marker;
      marker.header.frame_id = markerFrame_;
      marker.header.stamp = residualErrors_.header.stamp;
      marker.ns = "fp_marker_namespace";
      marker.id = 0;
      marker.type = MarkerT::TEXT_VIEW_FACING;
      marker.action = MarkerT::ADD;
      marker.pose.position.x = 0.0;
      marker.pose.position.y = -3.0;
      marker.pose.position.z = 0.0;
      marker.scale.x = 0.0;
      marker.scale.y = 0.0;
      marker.scale.z = 2.0;
      marker.text = "Failure Probability: " +
                    std::to_string((int)(failureProbability_ * 100.0)) + " %";
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
      if (failureProbability_ > 0.1)
        marker.color.r = marker.color.g = 0.0;
      failureProbabilityMarkerPub_->publish(marker);
    }
  }

  void printFailureProbability(void) {
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Failure probability = " << failureProbability_ * 100.0
                                                << " [%]");
  }

private:
  void residualErrors_callback(const LaserT::ConstSharedPtr &msg) {
    if (canUpdateResidualErrors_)
      residualErrors_ = *msg;
    if (!gotResidualErrors_)
      gotResidualErrors_ = true;
  }

  inline double calculateNormalDistribution(double e) {
    return (0.95 * (2.0 * NDNormConst_ *
                    exp(-((e - NDMean_) * (e - NDMean_)) / (2.0 * NDVar_))) +
            0.05 * (1.0 / maxResidualError_)) *
           residualErrorReso_;
  }

  inline double calculateExponentialDistribution(double e) {
    return (0.95 * (1.0 / (1.0 - exp(-EDLambda_ * maxResidualError_))) *
                EDLambda_ * exp(-EDLambda_ * e) +
            0.05 * (1.0 / maxResidualError_)) *
           residualErrorReso_;
  }

  inline double calculateUniformDistribution(void) {
    return (1.0 / maxResidualError_) * residualErrorReso_;
  }

  inline double getSumOfVecotr(std::vector<double> vector) {
    double sum = 0.0;
    for (int i = 0; i < (int)vector.size(); i++)
      sum += vector[i];
    return sum;
  }

  inline std::vector<double> getHadamardProduct(std::vector<double> vector1,
                                                std::vector<double> vector2) {
    for (int i = 0; i < (int)vector1.size(); i++)
      vector1[i] *= vector2[i];
    return vector1;
  }

  inline std::vector<double> normalizeVector(std::vector<double> vector) {
    double sum = getSumOfVecotr(vector);
    for (int i = 0; i < (int)vector.size(); i++)
      vector[i] /= sum;
    return vector;
  }

  inline double getEuclideanNormOfDiffVectors(std::vector<double> vector1,
                                              std::vector<double> vector2) {
    double sum = 0.0;
    for (int i = 0; i < (int)vector1.size(); i++) {
      double diff = vector1[i] - vector2[i];
      sum += diff * diff;
    }
    return sqrt(sum);
  }

  inline std::vector<double>
  calculateTransitionMessage(std::vector<double> probs) {
    std::vector<double> message(3);
    std::vector<double> tm = transitionProbMat_;
    message[ALIGNED] = tm[ALIGNED] * probs[ALIGNED] +
                       tm[MISALIGNED] * probs[MISALIGNED] +
                       tm[UNKNOWN] * probs[UNKNOWN];
    message[MISALIGNED] = tm[ALIGNED + 3] * probs[ALIGNED] +
                          tm[MISALIGNED + 3] * probs[MISALIGNED] +
                          tm[UNKNOWN + 3] * probs[UNKNOWN];
    message[UNKNOWN] = tm[ALIGNED + 6] * probs[ALIGNED] +
                       tm[MISALIGNED + 6] * probs[MISALIGNED] +
                       tm[UNKNOWN + 6] * probs[UNKNOWN];
    return message;
  }

  std::vector<std::vector<double>>
  getLikelihoodVectors(std::vector<double> validResidualErrors) {
    std::vector<std::vector<double>> likelihoodVectors(
        (int)validResidualErrors.size());
    double pud = calculateUniformDistribution();
    for (int i = 0; i < (int)likelihoodVectors.size(); i++) {
      likelihoodVectors[i].resize(3);
      likelihoodVectors[i][ALIGNED] =
          calculateNormalDistribution(validResidualErrors[i]);
      likelihoodVectors[i][MISALIGNED] =
          calculateExponentialDistribution(validResidualErrors[i]);
      likelihoodVectors[i][UNKNOWN] = pud;
      likelihoodVectors[i] = normalizeVector(likelihoodVectors[i]);
    }
    return likelihoodVectors;
  }

  std::vector<std::vector<double>> estimateMeasurementClassProbabilities(
      std::vector<std::vector<double>> likelihoodVectors) {
    std::vector<std::vector<double>> measurementClassProbabilities =
        likelihoodVectors;
    for (int i = 0; i < (int)measurementClassProbabilities.size(); i++) {
      for (int j = 0; j < (int)measurementClassProbabilities.size(); j++) {
        if (i == j)
          continue;
        std::vector<double> message =
            calculateTransitionMessage(likelihoodVectors[j]);
        measurementClassProbabilities[i] =
            getHadamardProduct(measurementClassProbabilities[i], message);
        measurementClassProbabilities[i] =
            normalizeVector(measurementClassProbabilities[i]);
      }
      measurementClassProbabilities[i] =
          normalizeVector(measurementClassProbabilities[i]);
    }

    double variation = 0.0;
    int idx1 = rand() % (int)measurementClassProbabilities.size();
    std::vector<double> message(3);
    message = likelihoodVectors[idx1];
    int checkStep = maxLPBComputationNum_ / 20;
    for (int i = 0; i < maxLPBComputationNum_; i++) {
      int idx2 = rand() % (int)measurementClassProbabilities.size();
      int cnt = 0;
      for (;;) {
        if (idx2 != idx1)
          break;
        idx2 = rand() % (int)measurementClassProbabilities.size();
        cnt++;
        if (cnt >= 10)
          break;
      }
      message = calculateTransitionMessage(message);
      message = getHadamardProduct(likelihoodVectors[idx2], message);
      std::vector<double> measurementClassProbabilitiesPrev =
          measurementClassProbabilities[idx2];
      measurementClassProbabilities[idx2] =
          getHadamardProduct(measurementClassProbabilities[idx2], message);
      measurementClassProbabilities[idx2] =
          normalizeVector(measurementClassProbabilities[idx2]);
      double diffNorm =
          getEuclideanNormOfDiffVectors(measurementClassProbabilities[idx2],
                                        measurementClassProbabilitiesPrev);
      variation += diffNorm;
      if (i >= checkStep && i % checkStep == 0 && variation < 10e-6)
        break;
      else if (i >= checkStep && i % checkStep == 0)
        variation = 0.0;
      message = measurementClassProbabilities[idx2];
      idx1 = idx2;
    }
    return measurementClassProbabilities;
  }

  double predictFailureProbabilityBySampling(
      std::vector<std::vector<double>> measurementClassProbabilities) {
    int failureCnt = 0;
    for (int i = 0; i < samplingNum_; i++) {
      int misalignedNum = 0, validMeasurementNum = 0;
      int measurementNum = (int)measurementClassProbabilities.size();
      for (int j = 0; j < measurementNum; j++) {
        double darts = (double)rand() / ((double)RAND_MAX + 1.0);
        double validProb = measurementClassProbabilities[j][ALIGNED] +
                           measurementClassProbabilities[j][MISALIGNED];
        if (darts > validProb)
          continue;
        validMeasurementNum++;
        if (darts > measurementClassProbabilities[j][ALIGNED])
          misalignedNum++;
      }
      double misalignmentRatio =
          (double)misalignedNum / (double)validMeasurementNum;
      double unknownRatio = (double)(measurementNum - validMeasurementNum) /
                            (double)measurementNum;
      if (misalignmentRatio >= misalignmentRatioThreshold_ ||
          unknownRatio >= unknownRatioThreshold_)
        failureCnt++;
    }
    double p = (double)failureCnt / (double)samplingNum_;
    return p;
  }

  void setAllMeasurementClassProbabilities(
      std::vector<double> residualErrors,
      std::vector<std::vector<double>> measurementClassProbabilities) {
    measurementClassProbabilities_.resize((int)residualErrors.size());
    int idx = 0;
    for (int i = 0; i < (int)measurementClassProbabilities_.size(); i++) {
      measurementClassProbabilities_[i].resize(3);
      if (0.0 <= residualErrors[i] && residualErrors[i] <= maxResidualError_) {
        measurementClassProbabilities_[i] = measurementClassProbabilities[idx];
        idx++;
      } else {
        measurementClassProbabilities_[i][ALIGNED] = 0.00005;
        measurementClassProbabilities_[i][MISALIGNED] = 0.00005;
        measurementClassProbabilities_[i][UNKNOWN] = 0.9999;
      }
    }
  }

  std::vector<int> getResidualErrorClasses(void) {
    int size = (int)measurementClassProbabilities_.size();
    std::vector<int> residualErrorClasses(size);
    for (int i = 0; i < size; i++) {
      double alignedProb = measurementClassProbabilities_[i][ALIGNED];
      double misalignedProb = measurementClassProbabilities_[i][MISALIGNED];
      double unknownProb = measurementClassProbabilities_[i][UNKNOWN];
      if (alignedProb > misalignedProb && alignedProb > unknownProb)
        residualErrorClasses[i] = ALIGNED;
      else if (misalignedProb > alignedProb && misalignedProb > unknownProb)
        residualErrorClasses[i] = MISALIGNED;
      else
        residualErrorClasses[i] = UNKNOWN;
    }
    return residualErrorClasses;
  }
};

} // namespace als_ros