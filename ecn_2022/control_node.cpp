// mandatory includes for base code
#include <rclcpp/rclcpp.hpp>
#include <ecn_2022/eigen.h>

// include any thing required - do not forget to use the .hpp extension for ROS 2 files
#include <baxter_core_msgs/msg/joint_command.hpp>
#include <baxter_simple_sim/srv/jacobian.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

// some shortcuts for message classes
using baxter_core_msgs::msg::JointCommand;
using geometry_msgs::msg::Twist;
using baxter_simple_sim::srv::Jacobian;

// defining sampling time
const auto dt{0.1};

const std::vector<std::string> suffixes = {"_s0", "_s1", "_e0", "_e1", "_w0", "_w1", "_w2"};
std::string arm;

class ControlNode : public rclcpp::Node
{
public:
  ControlNode() : Node("control")
  {
    // init member variables such as publishers / subscribers / timers
    
    // TODO declare a parameter for the drone number
    arm = declare_parameter<std::string>("arm", "CR7 is the GOAT");

    // init command message for arm
    command_msg_.set__mode(2);
    command_msg_.command.resize(suffixes.size(), 0);
    for (auto &suffix: suffixes)
    {
      std::string name = arm + suffix;
      command_msg_.names.push_back(name);
    }

    velocity_setpoints_subscriber_ = this->create_subscription<Twist>(arm+"/ee_twist", 10, 
    std::bind(&ControlNode::velocity_setpoints_callback, this, std::placeholders::_1));

    jacobian_node.init(arm,"robot/limb/"+arm+"/jacobian");

    command_publisher_ = this->create_publisher<JointCommand>("robot/limb/"+arm+"/joint_command",10);

  }
  
private:
  // declare member variables such as publishers / subscribers / timers
  rclcpp::Subscription<Twist>::SharedPtr velocity_setpoints_subscriber_;
  rclcpp::Publisher<JointCommand>::SharedPtr command_publisher_;
  JointCommand command_msg_;

  ServiceNodeSync<Jacobian> jacobian_node;


  // declare callbacks that are too large to be a simple lambda function

  void velocity_setpoints_callback(Twist::SharedPtr msg){
    Jacobian::Request req;
    Jacobian::Response res;

    req.inverse = true;
    req.ee_frame = true;


    if (!jacobian_node.call(req,res)){
      return;
    }

    JacobianInverseCoefs Jinv_coeffs = res.jacobian;
    auto velocity_command = computeCommand(Jinv_coeffs, *msg);

    command_msg_.command = velocity_command;

    command_publisher_->publish(command_msg_);
  }


};



// boilerplate main function

int main(int argc, char** argv)
{   
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
