// include any thing required - do not forget to use the .hpp extension for ROS 2 files
#include "urdf/target.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


using namespace std::chrono_literals;
using geometry_msgs::msg::Twist;
using sensor_msgs::msg::LaserScan;

namespace ecn_2021
{

class ControlNode : public rclcpp::Node
{
  // helper class to see the tracked point in RViz
  TargetPublisher target_pub;

  // TODO declare member variables for publishers, subscribers, timers and others if needed
  std::string robot_name{"turtlebot2"};
  double distance_{0.5};

  // Store the latest laser scan message
  LaserScan::SharedPtr latest_scan_msg;

public:
  ControlNode(rclcpp::NodeOptions options)
    : Node("control", options)
  {
    // TODO init parameters: distance to track the target and robot name
    distance_= declare_parameter<double>("distance", 0.5);
    robot_name = declare_parameter<std::string>("robot_name", "turtlebot2");

    // TODO use target_pub to see the tracked point in RViz
    target_pub.init(this, robot_name);

    // TODO init whatever is needed for your node: publishers / subscribers / timers / member variables
    cmd_vel_publisher_ = this->create_publisher<Twist>("cmd_vel", 10);
    
    timer_ = this->create_wall_timer(100ms, std::bind(&ControlNode::move, this));

    scan_subscription_ = this->create_subscription<LaserScan>(
    "scan", 10, std::bind(&ControlNode::laserScanCallback, this, std::placeholders::_1));


  }
  
private:
  // Declaring
  rclcpp::Publisher<Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<LaserScan>::SharedPtr scan_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  Twist cmd_vel_;

  void move()
  {
    // find nearest target: finish the findClosest function
    const auto target{findClosest(latest_scan_msg)};
    if(!target.isValid()) return;

    // debug
    target_pub.publish(target);

    // TODO compute control law
    cmd_vel_.linear.x = target.computeVx(target.range);
    cmd_vel_.angular.z = target.computeOmegaZ();

    // TODO publish control law

    cmd_vel_publisher_->publish(cmd_vel_);

  }


  Target findClosest(LaserScan::SharedPtr msg) const
  {
    // a Target is a range [m] and an angle [rad], default to an invalid one
    Target closest;

    // TODO closest should correspond to the smallest measured range
    // as long as it is between range_min and range_max
    double min = msg->range_max;

    for (int i=0; i<msg->ranges.size(); i++)
    {
      if ((msg->ranges[i] > msg->range_min || msg->ranges[i] < msg->range_max) && msg->ranges[i]< min ){
        closest.range = msg->ranges[i];
        closest.angle = msg->range_min + i*msg->angle_increment;
        min = closest.range;
      }
    }
    return closest;
  }

  // Subscription callback for laser scan messages
  void laserScanCallback(const LaserScan::SharedPtr msg)
  {
    latest_scan_msg = msg;
  }


};
}


// boilerplate main function, nothing to do here
int main(int argc, char** argv)
{   
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ecn_2021::ControlNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
