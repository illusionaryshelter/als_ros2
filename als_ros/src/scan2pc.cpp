#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <als_ros/als_ros_utils.hpp>
class Scan2PC : public rclcpp::Node {
public:
  using LaserT = sensor_msgs::msg::LaserScan;
  using PointCloudT = sensor_msgs::msg::PointCloud;

private:
  std::string scanName_, pcName_;
  rclcpp::Subscription<LaserT>::SharedPtr scanSub_;
  rclcpp::Publisher<PointCloudT>::SharedPtr pcPub_;

public:
  Scan2PC(void)
      : Node("Scan2Pc"), scanName_("/scan"), pcName_("/scan_point_cloud") {

    this->declare_parameter("scan_name", scanName_);
    this->declare_parameter("pc_name", pcName_);

    GET_PARAM_DEBUG("scan_name", scanName_);
    GET_PARAM_DEBUG("pc_name", pcName_);

    scanSub_ = rclcpp::create_subscription<LaserT>(
        *this, scanName_, 1,
        std::bind(&Scan2PC::scanCB, this, std::placeholders::_1));
    pcPub_ = rclcpp::create_publisher<PointCloudT>(*this, pcName_, 1);
  }

  void scanCB(const LaserT::ConstSharedPtr &msg) {
    PointCloudT pc;
    pc.header = msg->header;
    for (int i = 0; i < (int)msg->ranges.size(); ++i) {
      double r = msg->ranges[i];
      if (r <= msg->range_min || msg->range_max <= r)
        continue;
      double t = msg->angle_min + (double)i * msg->angle_increment;
      double x = r * cos(t);
      double y = r * sin(t);
      geometry_msgs::msg::Point32 p;
      p.x = x;
      p.y = y;
      p.z = 0.0;
      pc.points.push_back(p);
    }
    pcPub_->publish(pc);
  }
}; // class Scan2PC

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Scan2PC>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
