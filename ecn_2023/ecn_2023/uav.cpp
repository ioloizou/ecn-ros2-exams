#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <ecn_2023/client_spinner.h>
#include <ecn_2023/srv/target.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;
using ecn_2023::srv::Target;
using geometry_msgs::msg::Twist;

constexpr auto dt{0.01};

class UAV : public rclcpp::Node
{
  // helper class to call a service in a synchronous way
  ServiceNodeSync<Target> client{this};

  // PI part
  double Kp{}, Ki{}, Kw{};
  double eix{}, eiy{};

  // TODO declare additional member variables if needed
  std::string drone_number{};

public:
  UAV() : Node("uav")
  {

    // control params
    Ki = declare_parameter("Ki", .1);
    Kp = declare_parameter("Kp", 10.);
    Kw = declare_parameter("Kw", 5.);

    // TODO declare a parameter for the drone number
    drone_number = declare_parameter<std::string>("drone_number", "it is a secret");

    // TODO init the service client
    client.init("/target");

    // TODO init publisher and other things
    publisher_ = this->create_publisher<Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(50ms, std::bind(&UAV::updateControl, this));

  }

  // TODO declare additional member functions if needed
  void updateControl(){
    Target::Request req;
    Target::Response res; 
    req.uav = "uav" + drone_number;

    if(!client.call(req,res))
    {
      std::cout<<req.uav<<std::endl;
      std::cout<<res.x<<std::endl;
      return;
    }
    std::cout<<"Bye"<<std::endl;

    // Integral error for x and y
    eix += res.x;
    eiy += res.y;

    // Compute desired velocity
    msg.linear.x = (Kp*res.x + Ki*eix)*dt;
    msg.linear.y = (Kp*res.y + Ki*eiy)*dt;
    msg.linear.z = Kp*res.z*dt;
    msg.angular.z = Kw*res.theta*dt;

    std::cout<<msg.linear.x<<" "<<msg.linear.y<<" "<<msg.linear.z<<" "<<msg.angular.z<<std::endl;

    publisher_->publish(msg);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<Twist>::SharedPtr publisher_;
  Twist msg;

};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UAV>());
  rclcpp::shutdown();
}
